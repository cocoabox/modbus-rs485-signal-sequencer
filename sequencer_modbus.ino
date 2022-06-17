//#define MODBUS_DEBUG
//#define MODBUS_USE_HARDWARE_SERIAL

#include <SoftwareSerial.h>
#include <fastio.h>
#include <ACS712.h>

#include "src/ModbusRTUSlaveArduino/ModbusRTUSlave.h"

#include "constants.h"
#include "hardware_config.h"
#include "tones.h"

#include "programs/p1_flash.h"
#include "programs/p2_ped.h"
#include "programs/p3_ryg.h"

//
// PC  ---MAX485---  Arduino
// + ..A     DE,RE ..D2
// - ..B     RO    ..D3
//           DI    ..D4

#define VERSION 1

#define RELAY_OFF (RELAY_ON == LOW ? HIGH : LOW)

// this constant has to be named "DATA_SIZE" for the library to work
//                                   ||
uint16_t holding_register_values[DATA_SIZE] = {0};
SoftwareSerial my_serial(MAX485_RO_PIN, MAX485_DI_PIN);
ModbusRTUSlave *rtu;
hc165<
    LAST_HC165_OUTPUT_PIN, // 終段の74HC165のQHピン(#9)と接続するArduinoのピンの番号。
    ALL_HC165_CLOCK_PIN,   // 全ての74HC165のCLKピン(#2)と接続するArduinoのピンの番号。
    ALL_HC165_LOAD_PIN,    // 全ての74HC165のSH/LDピン(#1) と接続するArduinoのピン番号。
    2,                     // カスケード接続する74HC595の個数。
    MSBFIRST, HIGHBYTEFIRST>
    my165;

uint32_t next_measure_time = 0;

void die()
{
    tone(BUZZER_PIN, NOTE_A1);
    while (true)
        ; // freeze program
}
void coin_sound()
{
    tone(BUZZER_PIN, NOTE_B5, 100);
    delay(100);
    tone(BUZZER_PIN, NOTE_E6, 850);
    delay(800);
    noTone(BUZZER_PIN);
}

void one_up_sound()
{
    tone(BUZZER_PIN, NOTE_E6, 125);
    delay(130);
    tone(BUZZER_PIN, NOTE_G6, 125);
    delay(130);
    tone(BUZZER_PIN, NOTE_E7, 125);
    delay(130);
    tone(BUZZER_PIN, NOTE_C7, 125);
    delay(130);
    tone(BUZZER_PIN, NOTE_D7, 125);
    delay(130);
    tone(BUZZER_PIN, NOTE_G7, 125);
    delay(125);
    noTone(BUZZER_PIN);
}

void fireball_sound()
{
    tone(BUZZER_PIN, NOTE_G4, 35);
    delay(35);
    tone(BUZZER_PIN, NOTE_G5, 35);
    delay(35);
    tone(BUZZER_PIN, NOTE_G6, 35);
    delay(35);
    noTone(BUZZER_PIN);
}

uint8_t slave_address = 0;
uint8_t num_channels = 0;
uint8_t playback_switch = 0;
bool playback_switch_flipped = false;

static uint8_t channel_states[6] = {OFF};
static uint8_t last_channel_states[6] = {OFF};
static uint8_t last_relay_values[6] = {RELAY_OFF};

static uint8_t *current_program = NULL;
static uint8_t current_channel_count = 0;
static uint8_t current_step_count = 0;

static uint32_t next_step_time = 0;
static uint32_t next_flash0_time = 0;
static uint32_t next_flash1_time = 0;
static uint32_t next_flash2_time = 0;
static int step = 0;
static bool flash0 = false;
static bool flash1 = false;
static bool flash2 = false;

static bool just_restarted = false;
static uint8_t last_remote_mode = LOCAL_MODE;

//  TYPE  mV per Ampere
//  5A    185
//  20A   100  -----------------------.
//  30A   66                          |
ACS712 acs712(ACS712_PIN, 5.0, 1023, 100);

uint8_t read_switches(bool is_startup)
{
    uint16_t both_chips;
    both_chips = my165.shiftIn(); // 最も単純な出力方法。全16ビットの情報を一挙に74HC165から読み取る。
    uint8_t sw1 = both_chips & 0b1111;
    uint8_t sw2 = (both_chips & 0b11110000) >> 4;
    uint8_t cur_slave_address = sw1 * 10 + sw2;
    uint8_t cur_num_channels = (both_chips & 0b111100000000) >> 8;
    uint8_t cur_playback_switch = (both_chips & 0b1000000000000) >> 12;

    if (cur_playback_switch != playback_switch)
    {
        playback_switch_flipped = true;
    }
    else
    {
        playback_switch_flipped = false;
    }
    playback_switch = cur_playback_switch;

    if (!is_startup)
    {
        if (cur_slave_address != slave_address || cur_num_channels != num_channels)
        {
            return READ_SWITCH_CHANGED;
        }
        else
        {
            return READ_SWITCH_OK;
        }
    }
    if (cur_slave_address == 0 || cur_num_channels == 0)
    {
        return READ_SWITCH_INVALID_VALUES;
    }
    // assign global values
    slave_address = cur_slave_address;
    num_channels = cur_num_channels;
    return READ_SWITCH_OK;
}

void all_channels_off()
{
    memset(channel_states, OFF, sizeof(channel_states));
}

void advance_to_next_step()
{
    step++;
    if (step >= current_step_count)
        step = 0;
    next_step_time = millis() + prog_sec(step) * 1000;
}

bool set_current_program()
{
    switch (num_channels)
    {
    case 1:
        current_program = p1_program;
        current_channel_count = p1_num_channels;
        current_step_count = p1_num_steps;
        return true;
    case 2:
        current_program = p2_program;
        current_channel_count = p2_num_channels;
        current_step_count = p2_num_steps;
        return true;
    case 3:
        current_program = p3_program;
        current_channel_count = p3_num_channels;
        current_step_count = p3_num_steps;
        return true;
    default:
        return false;
    }
}

uint8_t *read_program(const uint8_t step, const int offset)
{
    return pgm_read_byte_near(current_program + (step * (current_channel_count + 1) + offset));
}

/**
 * get number of sec of a particualar step
 *
 * @param {int} step    step number, 0-based
 * @return {uint8_t}    number of second of step in sec ; returns 0 if error or no such step
 */
uint8_t prog_sec(const int step)
{
    if (step >= current_step_count)
        return 0;
    // ^ prevents reading out of bounds
    return read_program(step, 0);
}

/**
 * @param {int} step 0-based step number
 * @param {int} ch   0-based channel number
 * @return {int}  0=off, 1=on, 2=flash0, 3=flash1, 4=flash2
 */
int prog_ch_state(const int step, const int ch)
{
    if (step >= current_step_count)
        return 0;
    return read_program(step, 1 + ch);
}

int ch_state2relay(int ch_state)
{
    switch (ch_state)
    {
    case ON:
        return RELAY_ON;
    case FLASH0:
        return flash0 ? RELAY_ON : RELAY_OFF;
    case FLASH1:
        return flash1 ? RELAY_ON : RELAY_OFF;
    case FLASH2:
        return flash2 ? RELAY_ON : RELAY_OFF;
    case FLASH0_INVERT:
        return flash0 ?RELAY_OFF : RELAY_ON;
    case FLASH1_INVERT:
        return flash1 ?RELAY_OFF : RELAY_ON;
    case FLASH2_INVERT:
        return flash2 ? RELAY_OFF : RELAY_ON;
    case OFF:
    default:
        return RELAY_OFF;
    }
}

void restart_program()
{
    step = 0;
    next_step_time = millis() + prog_sec(0) * 1000;
    for (int i = 0; i < current_channel_count; ++i)
    {
        channel_states[i] = prog_ch_state(step, i);
    }
    just_restarted = true;
}

static int current_measurements[CURRENT_MEASUREMENT_COUNT] = {0};
static int current_measurements_idx = 0;

#define NO_MEASUREMENT_YET -1

int measure_current()
{
    int readout = acs712.mA_AC();
    current_measurements[current_measurements_idx] = readout;
    current_measurements_idx++;
    if (current_measurements_idx >= CURRENT_MEASUREMENT_COUNT)
    {
        current_measurements_idx = 0;
        int sum = 0;
        for (int i = 0; i < CURRENT_MEASUREMENT_COUNT; ++i)
        {
            sum += current_measurements[i];
        }
        long avg = sum / CURRENT_MEASUREMENT_COUNT;
        return avg / 100 * 100;
    }
    else
    {
        return NO_MEASUREMENT_YET;
    }
}

void setup()
{
    Serial.begin(9600);
    pinMode(ACS712_PIN, INPUT);
    acs712.autoMidPoint();

    pinMode(BUZZER_PIN, OUTPUT); // see also: https://soundapart.com/connect-8-ohm-speaker-arduino/
    my165.init();                // 74HC595とそのドライバの初期化

    if (read_switches(true) == READ_SWITCH_INVALID_VALUES)
    {
        Serial.println("invalid dip switch values");
        tone(BUZZER_PIN, 500);
        while (true)
            ; // freeze program
    }

    Serial.print("Modbus device ");
    Serial.print(slave_address);
    Serial.print("\nslave_address = ");
    Serial.print(slave_address);
    Serial.print("; num channels = ");
    Serial.print(num_channels);
    Serial.print("\n");

    holding_register_values[REG_NUM_CHANNELS] = num_channels;

    // set "current_program" and "current_channel_count" based on "num_channels"
    if (!set_current_program())
    {
        Serial.println("no corresponding program");
        while (true)
        {
            tone(BUZZER_PIN, 1000);
            delay(300);
            noTone(BUZZER_PIN);
            delay(300);
        }
    }
    restart_program();
    if (!playback_switch)
    {
        all_channels_off();
    }

    rtu = new ModbusRTUSlave(slave_address, &my_serial, MAX485_DE_RE_PIN);

    coin_sound();

    // Start modbus slave
    rtu->addWordArea(START_ADDRESS, holding_register_values, DATA_SIZE);
    rtu->begin(9600);
    holding_register_values[0] = VERSION;

    for (int i = 0; i < MAX_CHANNELS; ++i)
    {
        pinMode(RELAY_START_PIN + i, OUTPUT);
    }
}

void loop()
{
    uint32_t now = millis();
    bool need_redraw = false;
    if (just_restarted)
    {
        Serial.println("just restarted");
        need_redraw = true;
        just_restarted = false;
    }
    // handle flashers
    bool flash0_updated = false;
    bool flash1_updated = false;
    bool flash2_updated = false;

    if (now >= next_flash0_time)
    {
        flash0 = !flash0;
        flash0_updated = true;
        next_flash0_time = now + FLASH0_INTERVAL;
    }
    if (now >= next_flash1_time)
    {
        flash1 = !flash1;
        flash1_updated = true;
        next_flash1_time = now + FLASH1_INTERVAL;
    }
    if (now >= next_flash2_time)
    {
        flash2 = !flash2;
        flash2_updated = true;
        next_flash2_time = now + FLASH2_INTERVAL;
    }

    // read playback switch
    // but if CH or MODBUS_ID changed, then die until user resets
    if (read_switches(false) != READ_SWITCH_OK)
        die();

    rtu->process();

    if (holding_register_values[REG_REMOTE_MODE] == LOCAL_MODE)
    {
        bool just_started_local_mode = false;
        if (last_remote_mode != LOCAL_MODE)
        {
            fireball_sound();
            just_started_local_mode = true;
        }
        if ((just_started_local_mode && playback_switch) || (playback_switch && playback_switch_flipped))
        {
            // playback switch flipped to ON; start program and force redraw
            restart_program();
            need_redraw = true;
            Serial.println("restart and redraw");
        }
        if ((just_started_local_mode && !playback_switch) || (!playback_switch && playback_switch_flipped))
        {
            // playback switch flipped to OFF; turn off all channels
            all_channels_off();
            need_redraw = true;
            Serial.println("turn off and redraw");
        }
        if (playback_switch)
        {
            if (now >= next_step_time)
            {
                Serial.println("ADVANCE TO next step");
                need_redraw = true;
                advance_to_next_step();

                // update channel states
                Serial.print("update channels:");
                for (int i = 0; i < current_channel_count; ++i)
                {
                    const uint8_t ch_state = prog_ch_state(step, i);
                    Serial.print(" #");
                    Serial.print(i);
                    Serial.print("=");
                    Serial.print(ch_state == ON ? "ON" : ch_state == OFF  ? "OFF"
                                                     : ch_state == FLASH0 ? "F0"
                                                     : ch_state == FLASH1 ? "F1"
                                                     : ch_state == FLASH2 ? "F2"
                                                     : ch_state == FLASH0_INVERT ? "F0I"
                                                     : ch_state == FLASH1_INVERT ? "F1I"
                                                     : ch_state == FLASH2_INVERT ? "F2I"
                                                                          : "??");
                    channel_states[i] = ch_state;
                    if (((ch_state == FLASH0_INVERT || ch_state == FLASH0) && flash0_updated) 
                    || ((ch_state == FLASH1_INVERT || ch_state == FLASH1) && flash1_updated) 
                    || ((ch_state == FLASH2_INVERT || ch_state == FLASH2) && flash2_updated) 
                    || last_channel_states[i] != ch_state)
                    {
                        need_redraw = true;
                    }
                }
                Serial.print("\n");
            }
            else
            {
                // detect flash
                for (int i = 0; i < current_channel_count; ++i)
                {
                    const uint8_t ch_state = prog_ch_state(step, i);
                    if (((ch_state == FLASH0_INVERT || ch_state == FLASH0) && flash0_updated) 
                    || ((ch_state == FLASH1_INVERT || ch_state == FLASH1) && flash1_updated) 
                    || ((ch_state == FLASH2_INVERT || ch_state == FLASH2) && flash2_updated) 
                    )
                    {
                        need_redraw = true;
                        Serial.println("has flashing 2; redraw");
                    }
                }
            }
        }
        last_remote_mode = LOCAL_MODE;
    }
    else if (holding_register_values[REG_REMOTE_MODE] == SLAVE_MANUAL_MODE)
    {
        if (last_remote_mode != SLAVE_MANUAL_MODE)
        {
            // things to do when first turning to slave manual mode
            fireball_sound();
        }
        // detect if any channels has changed
        for (int i = 0; i < MAX_CHANNELS; ++i)
        {
            if (holding_register_values[REG_CH_0 + i] != last_channel_states[i])
            {
                Serial.println("[slave] requested channels contain new value; redraw");
                need_redraw = true;
                break;
            }
        }
        for (int i = 0; i < MAX_CHANNELS; ++i)
        {
            // update channel states using user input
            channel_states[i] = holding_register_values[REG_CH_0 + i];
            if ((channel_states[i] == FLASH0 && flash0_updated) || (channel_states[i] == FLASH1 && flash1_updated) || (channel_states[i] == FLASH2 && flash2_updated))
            {
                // Serial.println("[slave] has flash ; redraw");
                need_redraw = true;
                break;
            }
            last_channel_states[i] = channel_states[i];
        }
        last_remote_mode = SLAVE_MANUAL_MODE;
    }

    if (need_redraw)
    {
        for (int i = 0; i < MAX_CHANNELS; ++i)
        {
            digitalWrite(RELAY_START_PIN + i, ch_state2relay(channel_states[i]));
            holding_register_values[REG_CH_0 + i] = channel_states[i];
        }
    }

    // read current
    if (now >= next_measure_time)
    {
        int measurement = measure_current();
        if (measurement != NO_MEASUREMENT_YET)
        {
            holding_register_values[REG_CURRENT_MEASUREMENT] = measurement;
        }
        next_measure_time = millis() + MEASURE_INTERVAL;
    }

    // if user changed some not-supposed-to-be-changed  values, restore them
    holding_register_values[REG_VERSION] = VERSION;
    holding_register_values[REG_NUM_CHANNELS] = num_channels;
}
