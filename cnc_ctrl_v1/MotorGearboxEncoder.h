    /*This file is part of the Makesmith Control Software.

    The Makesmith Control Software is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Makesmith Control Software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with the Makesmith Control Software.  If not, see <http://www.gnu.org/licenses/>.
    
    Copyright 2014-2017 Bar Smith*/
    
    #ifndef MotorGearboxEncoder_h
    #define MotorGearboxEncoder_h
    
    class MotorGearboxEncoder{
        public:
            MotorGearboxEncoder(const int& pwmPin, const int& directionPin1, const int& directionPin2, const int& encoderPin1, const int& encoderPin2, const unsigned long& loopInterval);
            Encoder    encoder;
            Motor      motor;
            float      cachedSpeed();
            void       write(const float& speed);
            void       computePID();
            void       setName(const char& newName);
            char       name();
            void       initializePID(const unsigned long& loopInterval);
            void       setPIDAggressiveness(float aggressiveness);
            void       setPIDValues(float KpV, float KiV, float KdV);
            void       setEncoderResolution(float resolution);
            float      computeSpeed();
            String     getPIDString();
            String     pidState();
        private:
            double     _targetSpeed;
            double     _currentSpeed;
            volatile double     _lastPosition;
            volatile double     _lastTimeStamp;
            float               _lastDistMoved;
            float               _RPM;
            char       _motorName;
            double     _pidOutput;
            PID        _PIDController;
            double     _Kp=0, _Ki=0, _Kd=0;
            float      _encoderStepsToRPMScaleFactor = 7364.0;   //6*10^7 us per minute divided by 8148 steps per revolution
    };

    #endif