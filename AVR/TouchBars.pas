program TouchBars;
//nicht umbenennen wg. externer Asm-Routinen!
//für KlangComputer2-Platine mit ATmega8

// 26.03.2012 ProcClock auf 8 MHz geändert! DynScan nicht getestet!
{
(c) by c't magazin und Carsten Meyer, cm@ctmagazin.de
MIDI-Keyboard mit MIDI-Merge, Minimal-Version für E-LAB AVR Pascal Demo

Portbelegung:
Port B: ManualScan wie MIDIvice,
PB0: Pin_ShiftClk
PB1: Data_PS (Bass TOS out wenn DEFINE Pedalsustain)
PB2: RuheKont
PB3: ArbKont_ShiftOut
PB4: ColReset_PercOut
PB5: Data_PS wenn DEFINE Pedalsustain

}

{$NOSHADOW}
{$TYPEDCONST OFF}
{ $W+ Warnings}            {Warnings on/off}

{Defines aktivieren durch Entfernen des 1. Leerzeichens!}
{Auswahl der Peripherie}

Device = mega8, VCC = 5;

Import SysTick;

//From System Import;

Define
  ProcClock      = 8000000;        {Hertz}
  SysTick        = 2;               {msec}
  StackSize      = $0040, iData;
  FrameSize      = $0040, iData;
//  TickTimer      = Timer1;

Implementation

{--------------------------------------------------------------}
{ Type Declarations }
{--------------------------------------------------------------}
{ Const Declarations }
const

  DDRBinit                   = %00000011;            {PortB dir, PB2=OCR1A }
  PortBinit                  = %11111100;            {PortB }

  DDRCinit                   = %00110111;            {PortC dir }
  PortCinit                  = %00000000;            {PortC }

  DDRDinit                   = %11111111;            {PortD dir }
  PortDinit                  = %00000000;            {PortD }

{Lineal     .----.----.----.----.----.----.----.----.----.----.----.---}
  Vers1Str                   = 'c''t TouchBars 0.1';    //Vers1

  high:        boolean       = true;
  low :        boolean       = false;

  LEDtranslate   : Table[0..15] of Word =(
  511,255,127,63,31,15,7,3,1,1,1,1,1,1,1,1);


{--------------------------------------------------------------}
{ Var Declarations }

var

{$DATA} {Schnelle Register-Variablen}
  i, j, k, m, t  : Byte;

// Scan-Inputs
  _comp[@PinB, 3]:  bit;

{$PData}
  Pin_ShiftClk[@PortB, 0]: bit;
  Pin_DataLoad[@PortB, 1]: bit;
  Pin_srdata[@PinB, 2]: bit;

  _LEDaux[@PortD, 2]     : bit; {Bit 2 ist LED, separat}

  LEDrow0[@PortD, 0]: bit;
  LEDrow1[@PortD, 1]: bit;
  LEDrow2[@PortD, 2]: bit;
  LEDrow3[@PortD, 3]: bit;
  LEDrow4[@PortD, 4]: bit;
  LEDrow5[@PortD, 5]: bit;
  LEDrow6[@PortD, 6]: bit;
  LEDrow7[@PortD, 7]: bit;
  LEDrow8[@PortC, 0]: bit;
  CounterClk[@PortC, 1]: bit;
  CounterRst[@PortC, 2]: bit;
  Testpin[@PortC, 4]: bit;

{$IDATA}  {Langsamere SRAM-Variablen}
  ScanSema: Byte;
  JohnsonState: byte;
  

  EraseTimer : SysTimer8;

{$IDATA}  {Langsamere SRAM-Variablen}


// 16 einzelne Reihen vom SR, akkumuliert, davon 9 benutzt
  SensorTable    : Table[0..255] of byte;
  SensorRow: Byte;

  TouchbarTable     : Table[0..15] of byte; {Tasten-Zustands-Array}
  LEDcolumns   : Table[0..15] of Word;
  


{--------------------------------------------------------------}
{ MIDI functions }

{$NOSAVE}

Interrupt TIMER2COMP;   {Timer2 Compare Match, Scanroutine}
begin
  inc(ScanSema);                  {Scan freigeben}
end;

procedure JohnsonReset;
begin
  CounterRst:= high;
  JohnsonState:= 0;
  CounterRst:= low;
end;

procedure JohnsonInc;
begin
  CounterClk:= high;
  inc(JohnsonState);
  CounterClk:= low;
end;

procedure TouchbarToLED(my_col, my_val: byte);
var my_word: word;
begin
// Balkenanzeige, oben beginnend
  my_word := ($FF00 shra my_val);
  settable(LEDcolumns, my_col, my_word);
end;

procedure ActivateSensorRow(my_row: byte; my_load: boolean);
begin
  Pin_ShiftClk:= low;
  Pin_DataLoad:= my_load;
  PortD:=0;
  LEDrow8:=low;
  disableInts;
  if my_row = 0 then
    LEDrow8:=high;
// kritisches Timing: alle Schieberegister parallel laden
    nop;
    nop;
    nop;
    nop;
    nop;
    nop;
    nop;
    nop;
    Pin_ShiftClk:= high;
  else
    incl(PortD, 8-my_row); // Set Bit "my_row"
// kritisches Timing: alle Schieberegister parallel laden
    nop;
    nop;
    nop;
    Pin_ShiftClk:= high;
  endif;
  enableInts;
  Pin_DataLoad:= low;
  Pin_ShiftClk:= low;
  PortD:=0;
  LEDrow8:=low;
end;

procedure ReadSensorRow(my_row: byte);
// Schieberegister lesen und Sensor-Tabelle einer Reihe updaten
var my_index: byte; my_word: word;
begin
// Precharge-Impuls
  ActivateSensorRow(my_row, false);
  ActivateSensorRow(my_row, true);
  for k:= 7 downto 0 do  // Reihe mit 9 Sensoren lesen
    my_index:= (my_row shl 4) + k;
    if Pin_srdata then // Pegel high = Impuls fehlt, Sensor gedrückt
      t:= gettable(SensorTable, my_index);
      inctolim(t, 4);
      settable(SensorTable, my_index, t);
      if t > 2 then
        // Unterste Reihe = 8, Oberste Reihe = 0
        settable(TouchbarTable, k, my_row);
        TouchbarToLED(k, my_row);
      endif;
    endif;
    Pin_ShiftClk:= high;
    nop;
    Pin_ShiftClk:= low;
  endfor;
// Spalte rechts gesondert behandeln
  k:= 8;
  if Pin_srdata then // Pegel high = Impuls fehlt, Sensor gedrückt
    my_index:= (my_row shl 4) + k;
    t:= gettable(SensorTable, my_index);
    inctolim(t, 4);
    settable(SensorTable, my_index, t);
    if t > 2 then
      // Unterste Reihe = 8, Oberste Reihe = 0
      settable(TouchbarTable, k, my_row);
      TouchbarToLED(k, my_row);
    endif;
  endif;
end;


procedure ScanBtns;
var my_idx: byte; my_bool : boolean; my_word: word;
begin
  if JohnsonState = 9 then
// Spalte 10 (leer) erreicht
    for i:= 0 to 255 do
      settable(SensorTable, i, 0);
    endfor;

    Testpin:= high;
    nop;
    Testpin:= low;

// Sensor-Zustand lesen
    for SensorRow:= 0 to 8 do
      ReadSensorRow(SensorRow);
      ReadSensorRow(SensorRow);
      ReadSensorRow(SensorRow);
      ReadSensorRow(SensorRow);
    endfor;
    
// Jetzt wieder erste Spalte LEDs
    JohnsonReset;
  else
    JohnsonInc;
  endif;
  my_word:= gettable(LEDcolumns, JohnsonState);
  PortD:= lo(my_word);
  LEDrow8:= hi(my_word) and 1;
end;



{--------------------------------------------------------------}
{Menü-Prozeduren}


procedure initall;
{nach Reset aufgerufen}
begin
  DDRB:=  DDRBinit;            {PortB dir}
  PortB:= PortBinit;           {PortB}
  DDRC:=  DDRCinit;            {PortC dir}
  PortC:= PortCinit;           {PortC}
  DDRD:=  DDRDinit;            {PortD dir}
  PortD:= PortDinit;           {PortD}


  udelay(10); {für MatrixScan-Routine importieren}

  EnableInts;

  TCNT2:= 0;
  TCCR2:= %00001110;           {CTC mode, Prescaler=256}

  TIMSK:= TIMSK OR %10000000;  {OCIE2=1}

  udelay(10);
  OCR2:= 40;

  CounterRst:= low;
  CounterClk:= low;

  ScanSema:= 0;                 {Scan sperren bis Timer2-Interrupt}
  SetSysTimer(EraseTimer,10);
  for i:= 0 to 8 do
    TouchbarToLED(i, 0);
  endfor;

end;

{--------------------------------------------------------------}
{ Main Program }
{$IDATA}

begin
  Initall;

  loop             // universal ohne Koppeln
    if ScanSema > 0 then
  {Tastatur scannen wenn INT-Semaphore freigegeben und kein MIDI-Empfang aussteht}
      ScanBtns;
      SCanSema:= 0;
    endif;
  endloop;
end.

