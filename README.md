# Quadcopter
Quadcopter control is a fundamentally difficult and interesting problem. With six de-
grees of freedom (three translational and three rotational) and only four independent inputs
(rotor speeds), quadcopters are severely underactuated. In order to achieve six degrees of
freedom, rotational and translational motion are coupled. The resulting dynamics are highly
nonlinear, especially after accounting for the complicated aerodynamic effects. Finally, unlike
ground vehicles, helicopters have very little friction to prevent their motion, so they must pro-
vide their own damping in order to stop moving and remain stable. Together, these factors
create a very interesting control problem.
