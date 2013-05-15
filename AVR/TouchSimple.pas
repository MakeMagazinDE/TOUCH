program TouchSimple;
{
(c) by c't magazin und Carsten Meyer, cm@ctmagazin.de
}

{$NOSHADOW}
{ $W+ Warnings}            {Warnings on/off}


Device = mega8, VCC = 5;

Import Serport, SysTick;

//From System Import;

Define
  ProcClock      = 8000000;        {Hertz}
  SysTick        = 2;               {msec}
  StackSize      = $0040, iData;
  FrameSize      = $0040, iData;
  SerPort        = 9600, Stop1;    {Baud, StopBits|Parity}
  RxBuffer       = 32, iData;
  TxBuffer       = 32, iData;

Implementation

{--------------------------------------------------------------}
{ Type Declarations }
{--------------------------------------------------------------}
{ Const Declarations }
const
{$TYPEDCONST OFF}

  DDRBinit                   = %00101111;            {PortB dir, PB2=OCR1A }
  PortBinit                  = %00000110;            {PortB }

  DDRDinit                   = %00000100;            {PortD dir }
  PortDinit                  = %00000000;            {PortD }

  high:        boolean       = true;
  low :        boolean       = false;


{--------------------------------------------------------------}
{ Var Declarations }

var

{$DATA} {Schnelle Register-Variablen}
  i : Byte;

{$PData}
  _LED[@PortB, 1]     : bit; {Bit 2 ist LED, separat}

  DDR_D6[@DDRD, 6]: bit;
  Port_D6[@PortD, 6]: bit; // Ausgang

  DDR_D7[@DDRD, 7]: bit;
  Port_D7[@PortD, 7]: bit; // Ausgang
  SensorInp_D7[@PinD, 7]: bit;   // Eingang
  
{$IDATA}  {Langsamere SRAM-Variablen}

  sbase,sval : word;
  
  SensorTimer : SysTimer8;

{--------------------------------------------------------------}

procedure blink(my_blinks: byte; my_delay: word);
begin
  for i := 0 to my_blinks do
    _LED := low;
    mdelay(my_delay);
    _LED := high;
    mdelay(my_delay);
  endfor;
end;


function getSensor_D7: word;
var my_loopcount: word;
// Charge-Transfer-Sensor mit zwei I/Os Port D6 und D7
begin
  // S1 und S3 on, Kondensator wird entladen
  Port_D7:= low; // Sampling-Pin
  Port_D6:= low; // Drive-Pin
  DDR_D7:= high; // Sampling-Pin Datenrichtung: Ausgang
  DDR_D6:= high; // Drive-Pin Datenrichtung: Ausgang
  udelay(1);     // µs-Pause, Sampling-Kondensator entladen
  DDR_D7:= low;  // Datenrichtung: Eingang, beide offen
  DDR_D6:= low;
  // zunächst beide Schalter offen
  for my_loopcount:= 0 to 1023 do
    // S2 on, Drive-Port High;
    Port_D6:= high;
    DDR_D6:= high;
    nop;
    nop;
    // wieder kurz alle offen,
    // um Überschneidungen zu vermeiden:
    DDR_D6:= low;
    Port_D6:= low;
    nop;
    nop;
    // S1 on, Sample-Port low:
    DDR_D7:= high;
    nop;
    nop;

    if SensorInp_D7 then // Sample-Input auf High-Schwelle?
      break;             // Schleife abbrechen!
    endif;
  endfor;
  return(my_loopcount);  // Ergebnis
end;


procedure initall;
{nach Reset aufgerufen}
begin
  DDRB:=  DDRBinit;            {PortB dir}
  PortB:= PortBinit;           {PortB}
  DDRD:=  DDRDinit;            {PortD dir}
  PortD:= PortDinit;           {PortD}

  EnableInts;
  blink(5,100);
end;


{--------------------------------------------------------------}
{ Main Program }
{$IDATA}

begin
  Initall;
// Basiswert ermitteln
  sbase:= getSensor_D7;
  loop
// alle 20 ms alle Sensoren lesen
    if isSysTimerZero(SensorTimer) then
      sval:= sbase - getSensor_D7;
      setSysTimer(SensorTimer,10);
      _LED:= (sval > 20); // LEDs invertiert!
      write(serout, 'SVAL=');
      writeln(serout, IntToStr(sval));
    endif;
  endloop;
end.

