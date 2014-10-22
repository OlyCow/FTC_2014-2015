FTC_2014-2015
=============

This repository contains code for 'Oly Cow's 2014-2015 (Cascade Effect) robot.

There will (possibly) (at some point in the (distant) future) be complete documentation
for all the code contained here. Meanwhile, feel free to email team 'Oly Cow for any help
you might need (mooovingforward [at] gmail [dot] com).

We provide this code without any warranty that it will work. Of course, you don't care
because you don't have an exact replica of our robot. Furthermore, our robot is constantly
changing as the season goes on, and there is never a final version of our robot. As a
consequence, code from commits earlier in the season often cannot simply be rolled back.
Instead, the version control is there to provide a template or backup in case something
goes wrong which we need to fix.

Good luck, and we hope you will be able to use this code for improving your own designs.

### /Atmel/
This folder contains code which runs on AVRs for our custom sensor board. This contains a
complete implementation of the code for our "Sensory Overload" project and is dependent
on the custom circuit board in that repository. There is an older version (first iteration) of
code for our sensor board project, which can be found in our code repository for 2013-2014
(Block Party!), but we are undertaking a complete rewrite of the code to address issues
we observed while implementing our draft code.

### /RobotC/
This folder contains the code which is run on the NXT brick. This includes all of our tele-op
code, our autonomous programs, a copy of a custom wrapper library for RobotC functions,
and the NXT-backend for interpreting results from the communication link with our sensor
board. We also keep a copy of Xander's RobotC drivers so that none of our code will
break due to changes in Xander's drivers' code.

### /Documentation/
This folder contains (will contain? possibly? who am I kidding) general documentation
for using this repository. This might (in the future) include tutorials, overviews, and other helpful
documents for making the most of this repository, and extending/contributing to it.
