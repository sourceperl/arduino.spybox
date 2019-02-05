/*
  RS485 Modbus serial spy tool

  This sketch receives data over RS485 interface and outputs the data to the USB CDC Serial interface

  Hardware:
  - Arduino MKR Zero
  - Arduino MKR 485 shield (dip switch set as : A/B resisotor to off, half/full to on and Y/Z resistor off)
     -> RS signal to spy connect to A(+) and B(-)
*/

#include <stdarg.h>
#include <ArduinoRS485.h>
// from https://github.com/arkhipenko/TaskScheduler
#include <TaskScheduler.h>

// some const
// serial commands
#define CMD_TIMEOUT   10E3
#define MAX_CMD_SIZE  64
#define SERIAL_RX_PIN 13

// some global vars
// mode
bool dump_mode = false;
// rs rate info
char rs_parity = 'N';
uint32_t rs_baudrate = 9600;
uint16_t rs_eof_us = 3500;
// stats
uint32_t rs_stat_frame_count = 0;
uint32_t rs_stat_err_crc = 0;
uint32_t rs_stat_err_exp = 0;
uint32_t rs_stat_err_short = 0;
// task scheduler
Scheduler runner;

// some prototypes
void task_led_alive();
void task_serial_command();
void task_spy_rs();

// some tasks
Task t_cmd(1, TASK_FOREVER, &task_serial_command, &runner, true);
Task t_alive(100 * TASK_MILLISECOND, TASK_FOREVER, &task_led_alive, &runner, true);
Task t_spy(0, TASK_FOREVER, &task_spy_rs, &runner, true);

// tasks functions
void task_led_alive() {
  digitalWrite(LED_BUILTIN, ! digitalRead(LED_BUILTIN));
}

void task_serial_command() {
  // local static vars
  static String cmd_rx_buf = "";
  static bool cmd_echo_mode = false;
  static uint32_t cmd_t_last_char = 0;
  // check command
  while (Serial.available() > 0) {
    // receive loop
    while (true) {
      int inByte = Serial.read();
      // no more data
      if (inByte == -1)
        break;
      // manage backspace
      if ((inByte == 0x08) or (inByte == 0x7f)) {
        // remove last char in buffer
        cmd_rx_buf.remove(cmd_rx_buf.length() - 1);
        // send backspace + ' ' + backspace
        Serial.print((char) 8);
        Serial.print(' ');
        Serial.print((char) 8);
        break;
      }
      // if echo on
      if (cmd_echo_mode)
        Serial.print((char) inByte);
      // reset command buffer if the last rx is too old
      if (millis() - cmd_t_last_char > CMD_TIMEOUT)
        cmd_rx_buf = "";
      cmd_t_last_char = millis();
      // add data to s_cmd
      cmd_rx_buf += (char) inByte;
      // limit size to MAX_CMD_SIZE
      if (cmd_rx_buf.length() > MAX_CMD_SIZE)
        cmd_rx_buf.remove(0, cmd_rx_buf.length() - MAX_CMD_SIZE);
      // pause receive loop if \n occur
      if (inByte == '\n')
        break;
    }
    // skip command not ended with "\n"
    if (! cmd_rx_buf.endsWith("\n"))
      break;
    // remove leading and trailing \r\n, force case
    cmd_rx_buf.trim();
    cmd_rx_buf.toLowerCase();
    // check for command argument (cmd [space char] [arg])
    int index_space  = cmd_rx_buf.indexOf(" ");
    String s_arg = "";
    String s_cmd = cmd_rx_buf;
    if (index_space != -1) {
      s_cmd = cmd_rx_buf.substring(0, index_space);
      s_arg = cmd_rx_buf.substring(index_space + 1);
      s_arg.trim();
      s_arg.toLowerCase();
    }
    // check command
    if (s_cmd.equals("?") or s_cmd.equals("help")) {
      Serial.println("-------------------------------------------------------------------------------");
      Serial.println("dump           turn on dump mode (= display modbus frame as hexadecimal values)");
      Serial.println("end            turn off dump mode");
      Serial.println("stat           view current RS485 modbus stat (nb of frames, CRC errors....)");
      Serial.println("clear          clear current RS485 modbus stat");
      Serial.println("echo on        turn on command echo");
      Serial.println("echo off       turn off command echo");
      Serial.println("echo           display current status");
      Serial.println("auto           view autodetected serial baudrate");
      Serial.println("serial         view current RS485 params");
      Serial.println("rate 9600      set RS485 baudrate to 9600 bauds (auto update EOF)");
      Serial.println("parity even    set RS485 parity to even (other choice are none or odd)");
      Serial.println("eof 3500       set modbus EOF (end of frame) to 3500 us (= 3.5 ms)");
      Serial.println("-------------------------------------------------------------------------------");
    }
    else if (s_cmd.equals("echo")) {
      if (s_arg.equals("on")) {
        Serial.println("echo mode is on");
        cmd_echo_mode = true;
      }
      else if (s_arg.equals("off")) {
        Serial.println("echo mode is off");
        cmd_echo_mode = false;
      }
      else {
        Serial.print("echo mode is ");
        Serial.println(cmd_echo_mode ? String("on") : String("off"));
      }
    }
    else if (s_cmd.equals("dump")) {
      Serial.println("dump mode is on");
      dump_mode = true;
    }
    else if (s_cmd.equals("end")) {
      Serial.println("dump mode is off");
      dump_mode = false;
    }
    else if (s_cmd.equals("clear")) {
      // clear counters
      rs_stat_frame_count = 0;
      rs_stat_err_crc = 0;
      rs_stat_err_exp = 0;
      rs_stat_err_short = 0;
      // console stats
      serial_printf("stats: frame count %06d | error CRC %06d | exception %06d | too short %06d\r\n", rs_stat_frame_count, rs_stat_err_crc, rs_stat_err_exp, rs_stat_err_short);
    }
    else if (s_cmd.equals("stat")) {
      // console stats
      serial_printf("stats: frame count %06d | error CRC %06d | exception %06d | too short %06d\r\n", rs_stat_frame_count, rs_stat_err_crc, rs_stat_err_exp, rs_stat_err_short);
    }
    else if (s_cmd.equals("auto")) {
      // console stats
      //serial_printf("RX is %d\r\n", digitalRead(13));
      serial_printf("start baudrate autodetect: please wait...\r\n");
      for (uint8_t t = 1; t <= 5; t++) {
        uint32_t baudrate = find_baudrate();
        if (baudrate == 0)
          serial_printf("try %d: baudrate not found\r\n", t);
        else
          serial_printf("try %d: find baudrate at %d bauds\r\n", t, baudrate);
        delay(75);
      }
    }
    else if (s_cmd.equals("serial")) {
      rs485_show();
    }
    else if (s_cmd.equals("rate")) {
      uint32_t set_baudrate = s_arg.toInt();
      if (set_baudrate > 0) {
        rs_baudrate = set_baudrate;
        rs_eof_us = max(round((10.0 / rs_baudrate) * 3.5 * 1e6), 1);
      }
      rs485_setup();
    }
    else if (s_cmd.equals("parity")) {
      if (s_arg.equals("none") or s_arg.equals("n"))
        rs_parity = 'N';
      else if (s_arg.equals("even") or s_arg.equals("e"))
        rs_parity = 'E';
      else if (s_arg.equals("odd") or s_arg.equals("o"))
        rs_parity = 'O';
      rs485_setup();
    }
    else if (s_cmd.equals("eof")) {
      uint32_t set_eof = s_arg.toInt();
      if (set_eof > 0) {
        rs_eof_us = max(set_eof, 1);
      }
      rs485_setup();
    }
    else {
      Serial.println("unknown command send \"?\" to view online help");
    }
    // reset for next one
    cmd_rx_buf = "";
  }
}

void task_spy_rs() {
  // local static vars
  static uint32_t t_last_byte = 0;
  static uint16_t frame_pos = 0;
  static uint8_t frame_buff[256];
  // detect EOF (end of frame => silent of 3.5 x byte transmit time)
  if ((micros() - t_last_byte > rs_eof_us) and (frame_pos > 0)) {
    // frame counter
    rs_stat_frame_count++;
    // check errors
    bool err_short = false;
    bool err_crc = false;
    bool err_except = false;
    // check frame size
    if (frame_pos >= 5) {
      // check crc
      uint16_t c_crc16 = crc16(frame_buff, frame_pos - 2);
      if (!((frame_buff[frame_pos - 2] ==  (c_crc16 & 0xff)) and (frame_buff[frame_pos - 1] == (c_crc16 >> 8))))
        err_crc = true;
      // modbus except
      else if (frame_buff[1] > 0x80)
        err_except = true;
    } else {
      err_short = true;
    }
    // update stats
    if (err_crc)
      rs_stat_err_crc++;
    if (err_except)
      rs_stat_err_exp++;
    if (err_short)
      rs_stat_err_short++;
    if (dump_mode) {
      // update frame_status msg
      String frame_status = "";
      if (err_short)
        frame_status = "too short";
      else if (err_crc)
        frame_status = "bad CRC";
      else if (err_except)
        frame_status = "exception";
      // print header with frame size and CRC status
      serial_printf("MSG #%06d S%03d:", rs_stat_frame_count, frame_pos);
      // dump frame as hex value
      for (uint16_t i = 0; i < frame_pos; i++) {
        serial_printf(" %02x", frame_buff[i]);
      }
      // error message
      if (frame_status.length() > 0)
        serial_printf(" [%s]", frame_status.c_str());
      Serial.println();
    }
    frame_pos = 0;
  }
  // receive loop
  while (RS485.available() > 0) {
    // store byte in buffer
    frame_buff[frame_pos++] = RS485.read();
    // avoid buffer overflow
    if (frame_pos >= sizeof(frame_buff))
      frame_pos = 0;
    t_last_byte = micros();
  }
}

// some functions
uint16_t crc16(uint8_t *buf, uint8_t len)
{
  uint16_t crc = 0xFFFF;
  for (int pos = 0; pos < len; pos++)
  {
    crc ^= (uint16_t)buf[pos];
    for (int i = 8; i != 0; i--)
    {
      if ((crc & 0x0001) != 0)
      {
        crc >>= 1;
        crc ^= 0xA001;
      }
      else
        crc >>= 1;
    }
  }
  return crc;
}

uint32_t find_baudrate() {
  // detect symbol size: minimun pulse width
  uint32_t symbol_w_us = pulseIn(SERIAL_RX_PIN, LOW, 1E6);
  // normalize symbol width (in us) to baudrate
  if (symbol_w_us < 4)
    return 0;
  else if (symbol_w_us < 12)
    return 115200;
  else if (symbol_w_us < 20)
    return 57600;
  else if (symbol_w_us < 29)
    return 38400;
  else if (symbol_w_us < 40)
    return 28800;
  else if (symbol_w_us < 60)
    return 19200;
  else if (symbol_w_us < 80)
    return 14400;
  else if (symbol_w_us < 150)
    return 9600;
  else if (symbol_w_us < 300)
    return 4800;
  else if (symbol_w_us < 600)
    return 2400;
  else if (symbol_w_us < 1200)
    return 1200;
  else
    return 0;
}

void rs485_setup() {
  // begin with parity params
  if (rs_parity == 'E')
    RS485.begin(rs_baudrate, SERIAL_8E1);
  else if (rs_parity == 'O')
    RS485.begin(rs_baudrate, SERIAL_8O1);
  else
    RS485.begin(rs_baudrate, SERIAL_8N1);
  // turn receive on
  RS485.receive();
  // echo serial setup
  rs485_show();
}

void rs485_show() {
  // echo current setup on console
  Serial.println("RS serial set to " + String(rs_baudrate) + "," + rs_parity + ",8,1 | modbus EOF set to " + String(rs_eof_us) + " us");
}

void serial_printf(char *fmt, ... ) {
  char buf[256];
  va_list args;
  va_start (args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end (args);
  Serial.print(buf);
}

void setup() {
  // IO setup
  pinMode(LED_BUILTIN, OUTPUT);
  // init serial monitor (USB CDC)
  Serial.begin(9600);
  // wait monitor is connect
  //while (!Serial);
  // int RS485 serial
  rs485_setup();
}

void loop() {
  // scheduler handler
  runner.execute();
}

