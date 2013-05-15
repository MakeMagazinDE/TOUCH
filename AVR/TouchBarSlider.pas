program TouchBars;
// 3x QTouch-Slider an PortD
// Slider-Position über I2C Adr. $50 abfragbar (3 Bytes)
// Kann mit kostenloser Demo-Version von AVRco kompiliert werden (ATMega8/88-Version)


{$NOSHADOW}
{ $W+ Warnings}            {Warnings on/off}

{Defines aktivieren durch Entfernen des 1. Leerzeichens!}
{Auswahl der Peripherie}

Device = mega8, VCC = 5; // oder mega8 oder mega88

Import SPIdriver, Serport, SysTick, TWIslave;

//From System Import;

Define
  ProcClock      = 8000000;        {Hertz}
  SysTick        = 2;               {msec}
//  TWIpresc   = TWI_BR400;    // TWI speed}
  TWIaddr    = $50;            // TWI slave address
  TWIbuffer    = 8, iData;     // TWI rx/tx buffersize, location
  TWImode    = transparent;    // TWI handshake or transparent

  StackSize      = $0060, iData;
  FrameSize      = $0040, iData;
  SerPort        = 57600, Stop1;    {Baud, StopBits|Parity}
  RxBuffer       = 32, iData;
  TxBuffer       = 32, iData;
  SPIorder = MSB;
  SPImode = 0;        // 0..3
  SPIpresc = 1; // presc = 0..3 -> 4/16/64/128
  SPI_SS = false;
//  TickTimer      = Timer1;

Implementation

{--------------------------------------------------------------}
{ Type Declarations }
{--------------------------------------------------------------}
{ Const Declarations }
const
{$TYPEDCONST OFF}

  DDRBinit                   = %00101111;            {PortB dir, PB2=OCR1A }
  PortBinit                  = %00000110;            {PortB }

  DDRCinit                   = %00000000;            {PortC dir }
  PortCinit                  = %00000000;            {PortC }

  DDRDinit                   = %00000100;            {PortD dir }
  PortDinit                  = %00000000;            {PortD }

  PortD_on                   = %11110100;            {PortD Sensor und LED}
  PortD_off                  = %00000100;            {PortD LED}
  
{Lineal     .----.----.----.----.----.----.----.----.----.----.----.---}
  Vers1Str                   = 'c''t TouchBars 0.1';    //Vers1

  high:        boolean       = true;
  low :        boolean       = false;

  SensorScaleArray: Array [0..7] of Integer = (100,100,100,100,100,100,100,100);
  
// Bit-Reihenfolge oberes Nibble umgekehrt bei Prototyp
  LEDbarTable: Table[0..15] of byte = (
    $FF, $FE, $FC, $F8, $F0, $70, $30, $10, $00,
    0,0,0,0,0,0,0);
  LEDpointTable: Table[0..15] of byte = (
    $FF, $FE, $FD, $FB, $F7, $7F, $BF, $DF, $EF,
    0,0,0,0,0,0,0);

  c_sliderscale23: Integer = 11;
  c_slideroffset23: Integer = 7;
  c_sliderscale45: Integer = 10;
  c_slideroffset45: Integer = 7;
  c_sliderscale67: Integer = 12;
  c_slideroffset67: Integer = 7;
  c_touchthreshold: Integer = 18;
{--------------------------------------------------------------}
{ Var Declarations }

var

{$DATA} {Schnelle Register-Variablen}
  scale, base, sval, stemp, w  : integer;
  i : Byte;

{$PData}
  _LEDrd[@PortB, 1]     : bit; {Bit 2 ist LED, separat}
  _LEDgn[@PortB, 2]     : bit; {Bit 2 ist LED, separat}
  Testport[@PortB, 4]    : bit; {MISO Pin 9 an ISP}
  
  SR_data[@PortB, 3]    : bit;
  SR_clk[@PortB, 5]    : bit;

  DischargePin[@PortB, 0]: bit;
  
  SensorDirD6[@DDRD, 6]: bit;
  SensorLevelD6[@PortD, 6]: bit;
  
  SensorDirD7[@DDRD, 7]: bit;
  SensorLevelD7[@PortD, 7]: bit;
{$IDATA}  {Langsamere SRAM-Variablen}

  raw, filt, sum, diff, sval_temp: Integer;
  count, Lowpass_Integrate_Count: byte;
  slider67,
  slider45,
  Slider23: Integer;
  slider67_byte[@Slider67]: byte;
  slider45_byte[@Slider45]: byte;
  slider23_byte[@Slider23]: byte;
  SliderTouched23, sliderTouched45, sliderTouched67: boolean;
  AnySensorTouched: Boolean;

  SensorInpTemp: byte;
  SensorInpD4[@SensorInpTemp, 4]: bit;
  SensorInpD5[@SensorInpTemp, 5]: bit;
  SensorInpD6[@SensorInpTemp, 6]: bit;
  SensorInpD7[@SensorInpTemp, 7]: bit;

  SensorTimer : SysTimer8;

//  SensorScaleArray   : Array[0..7] of Integer;
  
  SensorRawArray    : Array[0..7] of Integer;
  SensorRaw0[@SensorRawArray+0]: Integer;
  SensorRaw1[@SensorRawArray+2]: Integer;
  SensorRaw2[@SensorRawArray+4]: Integer;
  SensorRaw3[@SensorRawArray+6]: Integer;
  SensorRaw4[@SensorRawArray+8]: Integer;
  SensorRaw5[@SensorRawArray+10]: Integer;
  SensorRaw6[@SensorRawArray+12]: Integer;
  SensorRaw7[@SensorRawArray+14]: Integer;

  SensorBaseArray    : Array[0..7] of Integer;
  SensorBase0[@SensorBaseArray+0]: Integer;
  SensorBase1[@SensorBaseArray+2]: Integer;
  SensorBase2[@SensorBaseArray+4]: Integer;
  SensorBase3[@SensorBaseArray+6]: Integer;
  SensorBase4[@SensorBaseArray+8]: Integer;
  SensorBase5[@SensorBaseArray+10]: Integer;
  SensorBase6[@SensorBaseArray+12]: Integer;
  SensorBase7[@SensorBaseArray+14]: Integer;

  SensorFlagArray    : Array[0..7] of Boolean;
  SensorFlag0[@SensorFlagArray+0]: Boolean;
  SensorFlag1[@SensorFlagArray+1]: Boolean;
  SensorFlag2[@SensorFlagArray+2]: Boolean;
  SensorFlag3[@SensorFlagArray+3]: Boolean;
  SensorFlag4[@SensorFlagArray+4]: Boolean;
  SensorFlag5[@SensorFlagArray+5]: Boolean;
  SensorFlag6[@SensorFlagArray+6]: Boolean;
  SensorFlag7[@SensorFlagArray+7]: Boolean;


// für SensorArrayLowpass
  SensorFilterArray    : Array[0..7] of Integer;
  SensorFilter0[@SensorFilterArray+0]: Integer;
  SensorFilter1[@SensorFilterArray+2]: Integer;
  SensorFilter2[@SensorFilterArray+4]: Integer;
  SensorFilter3[@SensorFilterArray+6]: Integer;
  SensorFilter4[@SensorFilterArray+8]: Integer;
  SensorFilter5[@SensorFilterArray+10]: Integer;
  SensorFilter6[@SensorFilterArray+12]: Integer;
  SensorFilter7[@SensorFilterArray+14]: Integer;

// Ergebnis-Arrays für 8 Sensoren
// Sensor berührt
  SensorTouchedArray    : Array[0..7] of Boolean;
  SensorTouched0[@SensorTouchedArray+0]: Boolean;
  SensorTouched1[@SensorTouchedArray+1]: Boolean;
  SensorTouched2[@SensorTouchedArray+2]: Boolean;
  SensorTouched3[@SensorTouchedArray+3]: Boolean;
  SensorTouched4[@SensorTouchedArray+4]: Boolean;
  SensorTouched5[@SensorTouchedArray+5]: Boolean;
  SensorTouched6[@SensorTouchedArray+6]: Boolean;
  SensorTouched7[@SensorTouchedArray+7]: Boolean;

// Sensor-Wert
  SensorValueArray    : Array[0..7] of Integer;
  SensorValue0[@SensorValueArray+0]: Integer;
  SensorValue1[@SensorValueArray+2]: Integer;
  SensorValue2[@SensorValueArray+4]: Integer;
  SensorValue3[@SensorValueArray+6]: Integer;
  SensorValue4[@SensorValueArray+8]: Integer;
  SensorValue5[@SensorValueArray+10]: Integer;
  SensorValue6[@SensorValueArray+12]: Integer;
  SensorValue7[@SensorValueArray+14]: Integer;

  SensorDiffArray    : Array[0..7] of Integer;
  SensorDiff0[@SensorDiffArray+0]: Integer;
  SensorDiff1[@SensorDiffArray+2]: Integer;
  SensorDiff2[@SensorDiffArray+4]: Integer;
  SensorDiff3[@SensorDiffArray+6]: Integer;
  SensorDiff4[@SensorDiffArray+8]: Integer;
  SensorDiff5[@SensorDiffArray+10]: Integer;
  SensorDiff6[@SensorDiffArray+12]: Integer;
  SensorDiff7[@SensorDiffArray+14]: Integer;

{--------------------------------------------------------------}
{ MIDI functions }

procedure SerCRLF;
begin
  serout(#$0D);
  serout(#$0A);
end;

procedure WriteInt(my_Hint: String[10]; my_int: Integer);
begin
  write(serout, my_Hint);
  write(serout, IntToStr(my_int));
  write(serout, ';');
end;

procedure SliderToLEDpoint(my_idx: byte);
var my_led:byte;
begin
  my_led:= gettable(LEDpointTable,my_idx);
  SPIoutByte(my_led);
end;

procedure SliderToLEDbar(my_idx: byte);
var my_led:byte;
begin
  my_led:= gettable(LEDbarTable,my_idx);
  SPIoutByte(my_led);
end;


procedure blink(my_blinks: byte; my_delay: word);
begin
  for i := 0 to my_blinks do
    _LEDrd := low;
    mdelay(my_delay);
    _LEDrd := high;
    mdelay(my_delay);
  endfor;
end;

procedure SensorArrayLowpass(my_idx: byte);
// Zu filternder Tabelleneintrag in my_idx
// simpler -6dB Lowpass
var my_sum1, my_sum2, my_delayed, my_freq: integer;
begin
  my_delayed:= SensorFilterArray[my_idx];
  my_sum1:= SensorRawArray[my_idx] - my_delayed;
  my_sum2:= (my_sum1 shra 3) + my_delayed;
  SensorFilterArray[my_idx]:= my_sum2;
end;

procedure SensorBaseLowpass(my_idx: byte);
// Zu filternder Tabelleneintrag in my_idx
// simpler -6dB Lowpass, hinter SensorArrayLowpass geschaltet
var my_sum1, my_sum2, my_delayed, my_freq: integer;
begin
  my_delayed:= SensorBaseArray[my_idx];
  my_sum1:= SensorFilterArray[my_idx] - my_delayed;
  my_sum2:= (my_sum1 shra 4) + my_delayed;
  SensorBaseArray[my_idx]:= my_sum2;
end;

{--------------------------------------------------------------}


procedure SensorLoop;
// Charge-Transfer mit Eingängen oberes Nibble
// und Schalttransistor an getrenntem Port
begin
// Flags auf "nicht erreicht" setzen
  SensorFlag0:= true;
  SensorFlag1:= true;
  SensorFlag2:= true;
  SensorFlag3:= true;
  SensorFlag4:= true;
  SensorFlag5:= true;
  SensorFlag6:= true;
  SensorFlag7:= true;
  
  // S1 und S3 on, Kondensator wird entladen
  PortD:= PortD_off;
  DischargePin:= high;
  DDRD:= PortD_on;
  udelay(1); // 10 µs Delay
  DischargePin:= low;
  DDRD:= PortD_off;
  // zunächst alle offen
  nop;
  nop;
  for w:= 0 to 1023 do
    // S2 on,  Sensor-Port High
    PortD:= PortD_on;
    DDRD:= PortD_on;
    nop;
    nop;

    // alle offen
    DDRD:= PortD_off;
    PortD:= PortD_off;
    nop;
    nop;

    // S1 on, Drive-Port low
    DischargePin:= high;
    nop;
    nop;
    
    SensorInpTemp:= PinD and $F0;
    
    if SensorFlag4 and SensorInpD4 then
      SensorRaw4:= w shla 4;
      SensorFlag4:= false;
    endif;

    if SensorFlag5 and SensorInpD5 then
      SensorRaw5:= w shla 4;
      SensorFlag5:= false;
    endif;

    if SensorFlag6 and SensorInpD6 then
      SensorRaw6:= w shla 4;
      SensorFlag6:= false;
    endif;

    if SensorFlag7 and SensorInpD7 then
      SensorRaw7:= w shla 4;
      SensorFlag7:= false;
    endif;

    DischargePin:= low;
    // alle wieder offen
  endfor;
end;

procedure initall;
{nach Reset aufgerufen}
begin
  DDRB:=  DDRBinit;            {PortB dir}
  PortB:= PortBinit;           {PortB}
//  DDRC:=  DDRCinit;            {PortC dir}
//  PortC:= PortCinit;           {PortC}
  DDRD:=  DDRDinit;            {PortD dir}
  PortD:= PortDinit;           {PortD}
  TWIsetSlaveAddr($50);
  EnableInts;
  write(serout, 'Proximity Sensor Test');
  SerCRLF;
end;

procedure Calibrate(my_channel,my_repeats: byte);
var my_count: byte; my_int: Integer;
{nach Reset aufgerufen}
begin
// Basiswert ermitteln
  write(serout, 'Calibration');
  SerCRLF;
  SensorLoop;
  mdelay(1);
  SensorLoop;
  // Filter-Array vorbelegen
  SensorFilterArray[my_channel]:= SensorRawArray[my_channel];
  _LEDrd := low;
  for my_count := 0 to my_repeats do
    mdelay(1);
    SensorLoop;
    SensorArrayLowpass(my_channel);
  endfor;
  _LEDrd := high;
  my_int:= SensorFilterArray[my_channel];
  SensorBaseArray[my_channel]:= my_int;
end;

{--------------------------------------------------------------}
{ Main Program }
{$IDATA}

begin
  Initall;
// Basiswert ermitteln
  Calibrate(4,50);
  Calibrate(5,50);
  Calibrate(6,50);
  Calibrate(7,50);
  blink(5,100);
  SliderToLEDbar(0);
  for i:= 0 to 7 do
    WriteInt('Scale '+char(i+48)+'=', SensorScaleArray[i]);
  endfor;
  SerCRLF;

  loop
  // alle 20 ms alle Sensoren lesen
    if isSysTimerZero(SensorTimer) then
      setSysTimer(SensorTimer,10);
      
      SensorLoop;
      AnySensorTouched:= false;
      for i:= 4 to 7 do
        SensorArrayLowpass(i);
        scale:= SensorScaleArray[i];
        base:=  SensorBaseArray[i];
        filt:= SensorFilterArray[i];
        raw:= SensorRawArray[i];

// differenziert, positive Werte bis ca. 100 bei Annäherung, negative bis ca -100 bei Entfernung
        diff:= filt - raw;
        SensorDiffArray[i]:= MulDivInt(diff, scale, 1000);  // = (diff * scale) / 1000

// Ergebnis Annäherung gefiltert
        sval:= base - filt;
        sval_temp:= MulDivInt(sval, scale, 1000);  // = (sval * scale) / 1000
        SensorValueArray[i]:= sval_temp;

// Ergebnis Annäherung ungefiltert, schnell
        sval:= base - raw;
        if MulDivInt(sval, scale, 1000) > c_touchthreshold then
// Sensor berührt, Einschalten verzögert
          if sval_temp > c_touchthreshold then
            SensorTouchedArray[i]:= true;
            AnySensorTouched:= true;
          endif;
        else
// Sensor nicht berührt, mit niedriger Frequenz Base-Wert updaten für Langzeit-Drift
          SensorTouchedArray[i]:= false;
          inc(Lowpass_Integrate_Count);
          if Lowpass_Integrate_Count = 63 then
            SensorBaseLowpass(i);
            Lowpass_Integrate_Count:= 0;
          endif;
        endif;
      endfor;

      sum:= SensorValue2 + SensorValue3;
      diff:= SensorValue2 - SensorValue3;
      SliderTouched23:= SensorTouched2 or SensorTouched3;
      if SliderTouched23 then
        Slider23:= c_slideroffset23 + MulDivInt(c_sliderscale23, diff, sum);
        if Slider23 < 0 then  // entspricht Slider45 < 0
          Slider23:= 0;
        endif;
        if Slider45 > 17 then
          Slider23:= 17;
        endif;
//        SliderToLEDpoint(Slider23_byte shr 1);
      endif;

      sum:= SensorValue4 + SensorValue5;
      diff:= SensorValue4 - SensorValue5;
      SliderTouched45:= SensorTouched4 or SensorTouched5;
      if SliderTouched45 then
        Slider45:= c_slideroffset45 + MulDivInt(c_sliderscale45, diff, sum);
        if Slider45 < 0 then  // entspricht Slider45 < 0
          Slider45:= 0;
        endif;
        if Slider45 > 17 then
          Slider45:= 17;
        endif;
        SliderToLEDpoint(Slider45_byte shr 1);
      endif;
      _LEDgn:= not SliderTouched45;  // LEDs invertiert!

      sum:= SensorValue6 + SensorValue7;
      diff:= SensorValue6 - SensorValue7;
      SliderTouched67:= SensorTouched6 or SensorTouched7;
      if SliderTouched67 then
        Slider67:= c_slideroffset67 + MulDivInt(c_sliderscale67, diff, sum);
        if Slider67 < 0 then  // entspricht Slider67 < 0
          Slider67:= 0;
        endif;
        if Slider67 > 17 then
          Slider67:= 17;
        endif;
        SliderToLEDbar(Slider67_byte shr 1);
      endif;
      _LEDrd:= not SliderTouched67;  // LEDs invertiert!

//      SliderToLED(byte(Slider67));
      TWItxBuffer[0]:= Slider23_byte;
      TWItxBuffer[1]:= Slider45_byte;
      TWItxBuffer[2]:= Slider67_byte;
      TWItxBuffer[3]:= byte(SliderTouched23);
      TWItxBuffer[4]:= byte(SliderTouched45);
      TWItxBuffer[5]:= byte(SliderTouched67);
      TWItxBuffer[6]:=TWIrxBuffer[6];
      TWItxBuffer[7]:=TWIrxBuffer[7];
      WriteInt('SL 23=', Slider23);
      WriteInt('SL 45=', Slider45);
      WriteInt('SL 67=', Slider67);
      SerCRLF;
    endif;
  endloop;
end.

