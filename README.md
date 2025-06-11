# rui
ultra simple cli ethercat motor control

rui -i <ethernet-device-name>

keyboard commands:

modes:

m + a    individual actuator motion
m + s    spin all found actuators
m + i    idle

select actuator:

s + 1    select 1st actuator
s + 2    select 2nd actuator
s + 3    select 3rd actuator
...

Arrow keys

up       in individual mode, increase position
down     in individual mode, decrease position
right    increase selected actuator
left     decrease selected actuator

