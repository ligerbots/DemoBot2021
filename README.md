# DemoBot2021
Code to control the WCD demo robot

The West Coast Drive demo robot has a tank drive with two motors on each side.
We will use the built-in SparkMax encoders for the odometry.
Since this is a demo robot, we don't have to be very precise.
The gearboxes are 11::1, so the 42 counts per revoilution from the SparkMax encoder translates to 462 counts fora full revolution of the the wheels.

We are using 6" wheels. The wheel diameter and the gear ratio should be defined in Constants

We will also need an xbox controller. The current driving setup from InfiniteRecharge2021 should be sufficient.
