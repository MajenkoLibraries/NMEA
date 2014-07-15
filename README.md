NMEA
====

Most GPS modules operate through TTL level RS-232 formatted serial.  The messages
they send usually conform to a standard called *NMEA*, or *National Marine Electronics
Association*.  These messages are in a relatively easy to parse format for microcontrollers,
but there are some often overlooked caveats, and the actual reception and identifying of the
messages in a raw serial data stream can be tricky unless you know what you are doing.

To this end the NMEA library has been written.  It takes raw serial data, parses it, identifies
the incoming messages, and parses them into meaningful values.

It is capable of receiving data through any serial connection based on the Stream class, so any
HardwareSerial object (Serial, Serial1, Serial2, etc), and any Stream based SoftwareSerial
objects.

Values are parsed all the time the `process()` function is called and stored internally until
required, and a full set of *getter* functions is provided to access all the data.
