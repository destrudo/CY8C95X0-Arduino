CY8C95X0
========

Arduino Driver libary for CY8C9560, CY8C9540 and CY8C9520 i2c-based I/O expander chips with integrated PWM circuits & interrupts.

These are interesting chips, they'd be good for large or small RGB LED cube projects, and as the code stands now it's perfectly capable of doing such things.

========
Notes:
========

I've tried to make this lib as small as possible, feel free to contact me if you have any recomendations or patches, read below to seriously get my attention, I don't pay much attention to email there.

I've done very little in the way of mapping out which pwm controller is used by any given pin, _getPortPwm is it, an array with values corresponding to pwm circuits.  If you happen to have a smaller or better way, by god please tell me

The ports are numbered and laid out in a rather strange fashion, so I've included some methods that will help, (digitalWrite(group,pin,mode), etc.)

This has only been tested on the 60 bit version, Soon I will determine whether or not it works on the 20 pin, but the 40 I will probably not test.  I think it should work perfectly fine.

The code is very well commented, before asking me, look in the source.  I know the examples I provided are minimal, Perhaps later I'll have some larger more complex ones that show everything.

========
Missing Functionality:
========

1) There's no proper user-level inversion handling, if you need that, I left one high level function that does very little...

2) There's no interrupt handling, just a placeholder that contains a write to the register like above.

3) Be wary of the PWM controller and the ports you use.  You'll need to manually switch on and off the outputs (Using digitalWrite) which are connected to the pwm controller you want to use, otherwise they'll all light up.

========
Bug requests:
========


If I don't respond to any bug requests, find me on freenode under the handle 'destrudo'.  Querying me will likely result in things getting fixed, or functionality expanded.


========
What's next:
========

Internal functions for easily handling pwm pin outputs

========
Commentary:
========


Never trust a chip.
