## Project: Building a Controller

---

# Required Scenarios for a Passing Submission:

## [Rubric](https://review.udacity.com/#!/rubrics/1643/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.

---

### Writeup

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.

You are reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

The code snippets for passing each scenario are seperated into different commits. Please use "git log --oneline" to check commit history.

### Implemented Controller

#### 1. Determine the standard deviation of the measurement noise of both GPS X data and Accelerometer X data. (Scenario 6)

- First, we execute the CPPEstSim and scenario 6, the log would be output at config/log/{Graph1,Graph2}.txt respectively.
- Then we can calculate the standard deviation from command line as such
```
$ cat config/log/Graph1.txt | awk -F',' '{print $2}' | awk '{delta = $1 - avg; avg += delta / NR; mean2 += delta * ($1 - avg); } END { print "std of GPS X data = " sqrt(mean2 / NR); }'
std of GPS X data = 0.667953

$ cat config/log/Graph2.txt | awk -F',' '{print $2}' | awk '{delta = $1 - avg; avg += delta / NR; mean2 += delta * ($1 - avg); } END { print "std of Accelerometer X data = " sqrt(mean2 / NR); }'
std of Accelerometer X data = 0.475746
```

#### 2. Implement a better rate gyro attitude integration scheme in the UpdateFromIMU() function.

- follow the hint gave in `void QuadEstimatorEKF::UpdateFromIMU(V3F, V3F);`, 
```
  //  - there are several ways to go about this, including:
  //    2) use the Quaternion<float> class, which has a handy FromEuler123_RPY function for creating a quaternion from Euler Roll/PitchYaw
  //       (Quaternion<float> also has a IntegrateBodyRate function, though this uses quaternions, not Euler angles)
```

- C++ implementation
```
  Quaternion<float> attitude = Quaternion<float>::FromEuler123_RPY(rollEst, pitchEst, ekfState(6));
  attitude.IntegrateBodyRate(gyro, dtIMU);

  float predictedPitch = attitude.Pitch();
  float predictedRoll = attitude.Roll();
  ekfState(6) = attitude.Yaw();

  // do not forget to comment out the given code
```

#### 3. Implement all of the elements of the prediction step for the estimator.

- there are 3 parts of it
  - MatrixXf QuadEstimatorEKF::GetRbgPrime(float, float, float);
  - VectorXf QuadEstimatorEKF::PredictState(VectorXf, float, V3Fl, V3F);
  - void QuadEstimatorEKF::Predict(float, V3F, V3F);

- Python reference implementation of GetRbgPrime
```
   def rotation_matrix(self,phi,theta):
        '''
        Returns the rotation matrix for the given roll, pitch and yaw angles. 
        '''
        
        psi = 0.0 
        r_x = np.array([[1, 0, 0],
                        [0, np.cos(phi), -np.sin(phi)],
                        [0, np.sin(phi), np.cos(phi)]])

        r_y = np.array([[np.cos(theta), 0, np.sin(theta)],
                        [0, 1, 0],
                        [-np.sin(theta), 0, np.cos(theta)]])

        r_z = np.array([[np.cos(psi), -np.sin(psi), 0],
                        [np.sin(psi), np.cos(psi), 0],
                        [0,0,1]])

        r = np.matmul(r_z,np.matmul(r_y,r_x))

        return r 
``` 

- C++ implementation of funciton GetRbgPrime
```
    float cos_theta = cos(pitch);
    float cos_phi = cos(roll);
    float cos_psi = cos(yaw);

    float sin_theta = sin(pitch);
    float sin_phi = sin(roll);
    float sin_psi = sin(yaw);
    
    RbgPrime(0,0) = - cos_theta * sin_psi;
    RbgPrime(0,1) = - cos_theta * cos_psi - sin_phi * sin_theta * sin_psi;
    RbgPrime(0,2) =   sin_phi   * cos_psi - cos_phi * sin_theta * sin_psi;
    
    RbgPrime(1,0) =   cos_theta * cos_psi;
    RbgPrime(1,1) = - cos_phi   * sin_psi + sin_phi * sin_theta * cos_psi ;
    RbgPrime(1,2) =   sin_phi   * sin_psi + cos_phi * sin_theta * cos_psi;
```

- Hint of function PredictState
```
  // - dt is the time duration for which you should predict. It will be very short (on the order of 1ms)
  //   so simplistic integration methods are fine here
  // - we've created an Attitude Quaternion for you from the current state. Use 
  //   attitude.Rotate_BtoI(<V3F>) to rotate a vector from body frame to inertial frame
```

- C++ implementation of function PredictState
```
  for(int i = 0; i < 3; i++)
    predictedState(i) += predictedState(i+3) * dt; 
  
  V3F vaccel = attitude.Rotate_BtoI(accel) - V3F(0, 0, CONST_GRAVITY);
  predictedState(3) += vaccel.x * dt;
  predictedState(4) += vaccel.y * dt;
  predictedState(5) += vaccel.z * dt;
```

- Reference of function Predict from [Estimation of Quadrotors](https://www.overleaf.com/project/5c34caab7ecefc04087273b9)
![](https://i.imgur.com/oIbkKHu.png)

- Hint of function Predict
```
  // - update the covariance matrix cov according to the EKF equation.
  // 
  // - you may find the current estimated attitude in variables rollEst, pitchEst, state(6).
  //
  // - use the class MatrixXf for matrices. To create a 3x5 matrix A, use MatrixXf A(3,5).
  //
  // - the transition model covariance, Q, is loaded up from a parameter file in member variable Q
```

- C++ implementation of function Predict
```
  gPrime(0,3) = dt;
  gPrime(1,4) = dt;
  gPrime(2,5) = dt;
  
  gPrime(3, 6) = dt * (RbgPrime(0) * accel).sum();
  gPrime(4, 6) = dt * (RbgPrime(1) * accel).sum();
  gPrime(5, 6) = dt * (RbgPrime(2) * accel).sum();
  
  ekfCov = Q + gPrime * ekfCov * gPrime.transpose();
```

#### 4. Implement the magnetometer update.

- follow the hint gave in `void QuadEstimatorEKF::UpdateFromMag(float);`
```
  //  - Your current estimated yaw can be found in the state vector: ekfState(6)
  //  - Make sure to normalize the difference between your measured and estimated yaw
```

- C++ implementation
```
  zFromX(0) = ekfState(6);
  float delYaw = z(0) - zFromX(0);

  if (delYaw < -F_PI) z(0) += 2.f * F_PI;
  if (delYaw >  F_PI) z(0) -= 2.f * F_PI;

  hPrime(0, 6) = 1;
```

#### 5. Implement the GPS update.

- this update is somehow trivial since the estimated parameters are stored in ekfState

- C++ implementation
```
  for (int i = 0; i < 6; i++)
    zFromX(i) = ekfState(i);
  
  for (int i = 0; i < 6; i++)
    hPrime(i, i) = 1.f;
```

### Flight Evaluation

#### 1. Meet the performance criteria of each step.
- Scenario 1 [PASS] Intro
- Scenario 2 [PASS] AttitudeControl
- Scenario 3 [PASS] PositionControl
- Scenario 4 [PASS] Nonidealities
- Scenario 5 [PASS] TrajectoryFollow
- Scenario 6 [PASS] NoisySensors
- Scenario 7 [PASS] AttitudeEstimation
- Scenario 8 [DEMO] PredictState
- Scenario 9 [DEMO] PredictionCov
- Scenario 10 [PASS] MagUpdate
- Scenario 11 [PASS] GPSUpdate
(Congratulations!  We are all Done!)

#### 2. De-tune your controller to successfully fly the final desired box trajectory with your estimator and realistic sensors.

- we have to de-tune or re-tune on position and velocity gain parameter when we integrate our previous controller project, here is what i have adjusted
```
kpPosXY: 33 -> 33
kpPosZ: 28 -> 31
kpVelXY: 12 -> 9
```

- flight evaluation snapshot
![](https://i.imgur.com/ShIhVgW.png)
