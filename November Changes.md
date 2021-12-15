# November Changes

## 11/6/21, 6:45:24am:

### A turn to position function was added to the mobile goal lift

Commit Hash: 711b5ffddc5afbb1a3f05267f4af87d5f74e8d37

**MogoLift.h:**

```c++
+        /**
+         * @brief 
+         * 
+         * @param    directionType       My Param doc
+         * @param    position            My Param doc
+         * @param    rotationUnits       My Param doc
+         * @param    velocity            My Param doc
+         * @param    velocityUnits       My Param doc
+         */
+        void turnToPosition(double position, vex::rotationUnits rotationUnits, double velocity, vex::velocityUnits velocityUnits);
```

**MogoLift.cpp:**

```c++
+        void MogoLift::turnToPosition(double position, vex::rotationUnits rotationUnits, double velocity, vex::velocityUnits velocityUnits) {
+               mogoMotor.spinToPosition(position, rotationUnits, velocity, velocityUnits);
+        }
```

------------------------------------

## 11/6/21, 11:48:56 am:

### Boilerplate code for an Odometry class was added

Commit Hash: 39465a61e2ceae74a721b4c3aa42732c5c328f89

**Odometry.h:**

```c++
+        /**
+         * @file Odometry.h
+         * @author your name (you@domain.com)
+         * @brief 
+         * @version 0.1
+         * @date 2021-10-30
+         * 
+         * @copyright Copyright (c) 2021
+         * 
+         */
+
+        #include "vex.h"
+
+        class Odometry {
+        public:
+                Odometry();
+                void update();
+
+        private:
+
+                double currentHeading;
+                double averageLeft;
+                double averageRight;
+
+                double previousHeading;
+                double previousAverageLeft;
+                double previousAverageRight;
+
+                
+        };
```

**Odometry.cpp:**

```c++
+        /**
+         * @file Odometry.cpp
+         * @author your name (you@domain.com)
+         * @brief 
+         * @version 0.1
+         * @date 2021-10-30
+         * 
+         * @copyright Copyright (c) 2021
+         * 
+         */
+
+        #include "control/Odometry.h"
+
+        Odometry::Odometry() {
+
+        }
+
+        void Odometry::update() {
+               this->previousHeading = this->currentHeading;
+               this->previousAverageLeft = this->averageLeft;
+               this->previousAverageRight = this->averageRight;
+
+
+        }
```

-------

## 11/6/21, 11:50:30 am:

### Some functions for auto balancing and stopping whenever a certain amount of g-force where added but we were never really able to test them

Commit Hash: 48e0d14b807461a533542ad90a8f380959969a59

**FourMotorDrive.h:**

```c++
        /**
         * @brief Drives the base in one direction at a set speed
         * 
         * @param    directionType       The direction the base should drive
         * @param    velocity            Sets the velocity
         * @param    velocityUnits       The units for velocity
         */
        void drive(vex::directionType directionType, double velocity, vex::velocityUnits velocityUnits);

+       void goToPosition(double targetX, double targetY);

        /**
         * @brief Drives the base a specific distance using PID
         * 
         * @param    directionType       The direction the base should drive
         * @param    target              The target distance it should drive in inches
         */
        void driveDistance(vex::directionType directionType, double target); 
```

```c++
        /**
         * @brief Set the brake type of the base
         * 
         * @param    brakeType           The type of brake
         */
        void setBrake(vex::brakeType brakeType);

+       /**
+        * @brief Balance on platform
+        * 
+        * @param    directionType       The direction to drive to balance on the platform
+         */
+       void balance(vex::directionType directionType);
+
+       /**
+        * @brief Drive until the accelerometer reads above the given value
+        * 
+        * @param    directionType       The direction to drive the base until it hits something
+        */
+       void driveUntilHit(vex::directionType directionType);

        /**
         * @brief Stop the base from moving
         * 
         */
        void stop();
```

```c++
        /**
         * @brief Get the average position of the motor encoders
         * 
         * @param    rotationUnits       The units of rotation to return
         * @return   double              The average rotation value of the motor encoders
         */
        double getAveragePosition(vex::rotationUnits rotationUnits);

+       /**
+        * @brief Get the Total G Force object
+        * 
+        * @return   double              My Return Doc 
+        */
+       double getTotalGForce();
+
+       /**
+        * @brief Get the Average G Force object
+        * 
+        * @return   double              My Return Doc 
+        */
+       double getAverageGForce();
+
        /**
         * @brief reset the motor encoders
         * 
         */
        void resetPosition();
```

**FourMotorDrive.cpp:**

```c++
        #include "movement/drive/FourMotorDrive.h"

+       #define ratio 1.5
+
        FourMotorDrive::FourMotorDrive(motor_group &right, motor_group &left, inertial &sensor) : right (right), left(left), sensor(sensor) {
        //test
        }
```

```c++
        //drive functions
        void FourMotorDrive::drive(vex::directionType directionType, double velocity, vex::velocityUnits velocityUnits) {
                right.spin(directionType, velocity, velocityUnits);
                left.spin(directionType, velocity, velocityUnits);
        }

+       void FourMotorDrive::goToPosition(double targetX, double targetY) {
+               double deltaX = targetX - GPS.xPosition(distanceUnits::in);
+               double deltaY = targetY - GPS.yPosition(distanceUnits::in);
+       
+               //turn to face point
+               FourMotorDrive::turnToHeading(180-tan(deltaX/deltaY));
+
+               //update position from turn drift
+               deltaX = targetX - GPS.xPosition(distanceUnits::in);
+               deltaY = targetY - GPS.yPosition(distanceUnits::in);
+
+               //drive distance from current position to point
+               FourMotorDrive::driveDistance(forward, sqrtf(deltaX*deltaX + deltaY*deltaY));
+       }

        void FourMotorDrive::driveDistance(vex::directionType directionType, double target) {
+               PIDController pid(3.7, 10, 0);
                pid.setTarget(target);
                pid.setTolerance(0.1);
                FourMotorDrive::resetPosition();
        
                while(!pid.atTarget()) {
                        //idk move or something
+                       double distance = FourMotorDrive::getAveragePosition(vex::rotationUnits::rev) * (M_PI * ((1.625 * 4) * ratio)) ;
                        Controller1.Screen.clearScreen();
                       Controller1.Screen.setCursor(1, 1);
                       Controller1.Screen.print(pid.getError());
                       FourMotorDrive::drive(directionType, pid.calculate(distance), velocityUnits::pct);
                }
                FourMotorDrive::stop();
        }

+       // ! No idea if works or not
        void FourMotorDrive::driveTime(double time, vex::timeUnits timeUnits, vex::directionType directionType, double velocity, vex::velocityUnits velocityUnits) {
                FourMotorDrive::drive(directionType, velocity, velocityUnits);
                wait(time, timeUnits);
                FourMotorDrive::stop();
        }
```

```c++
                        FourMotorDrive::setRightSide(forward, velocity, velocityUnits);
                }
        }

+       // * works sorta
        void FourMotorDrive::turnToHeading(double target) {
+               PIDController pid(0.4, 0.3/ 1.5, 0);
                pid.setTolerance(3);
                pid.setProportionalCutoff(4);
                int error = 100;
```

```c++
        void FourMotorDrive::setBrake(vex::brakeType brakeType) {
                right.setStopping(brakeType);
                left.setStopping(brakeType);
        }

+       void FourMotorDrive::balance(vex::directionType directionType) {
+               while(sensor.pitch(vex::rotationUnits::deg) < 20) {
+                       FourMotorDrive::drive(directionType, 100, velocityUnits::pct);
+               }
+       }
+
+       void FourMotorDrive::driveUntilHit(vex::directionType directionType) {
+               //while(FourMotorDrive::getTotalGForce())
+       }
+
        void FourMotorDrive::stop() {
                right.stop();
                left.stop();
        }
```

```c++
        double FourMotorDrive::getAveragePosition(vex::rotationUnits rotationUnits) {
                return (right.position(rotationUnits) + 
                        left.position(rotationUnits)) / 4;
        }

+       double FourMotorDrive::getTotalGForce() {
+               return sensor.acceleration(vex::axisType::xaxis) + sensor.acceleration(vex::axisType::yaxis) + sensor.acceleration(vex::axisType::zaxis);
+       }
+
+       double FourMotorDrive::getAverageGForce() {
+               return FourMotorDrive::getTotalGForce()/3;
+       }
+
        void FourMotorDrive::resetPosition() {
                right.resetPosition();
                left.resetPosition();
        }
```

# UNFINISHED COMMIT