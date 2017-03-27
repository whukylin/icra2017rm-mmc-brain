#ifndef SERIAL_H
#define SERIAL_H

int connect_serial( const char* device, int baudrate = 115200);

int read_serial( unsigned char* data, int max_size, int timeout );

void disconnect_serial();

#endif
