clc
clear all

serial = serialport('COM13',115200);

configureTerminator(serial,"CR/LF");
flush(serial);

data = readline(serial);

%output = read(serial,100,'uint16');
clear all