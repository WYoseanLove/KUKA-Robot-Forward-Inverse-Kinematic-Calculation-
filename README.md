KUKA_Robot_Kinematic_Calculation used to calculate the KUKA 6 DOF Industrial Robot forward Kinematic and inverse Kinematic.
From the class you can given input J1-J6 joint degree for KUKA Robot and will auto calculate the end effector posture, and same if you give the end effector posture, you can get the J1-J6 JOINT  degree for all the possible solutions.
The calculation is base on c++ Eigen Library, and all the calculation is deducted from J1 until J6.

Usage:          KUKA 6 DOF Industrial Robot forward Kinematic and inverse Kinematic, can be modified to other industry robot like ABB, FANUC and Kawasaki

References:     https://www.youtube.com/watch?v=C0H8XsSPGyo
                https://www.youtube.com/watch?v=NDEEKGEQylg

Math.NET License Info:  
               Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
               documentation files (the "Software"), to deal in the Software without restriction, including without limitation 
               the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and 
               to permit persons to whom the Software is furnished to do so, subject to the following conditions:

               THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO 
               THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
               AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
               TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.