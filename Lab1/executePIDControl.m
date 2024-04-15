function executePIDControl()
    global poseData
    persistent integral lastError lastTime
    if isempty(integral)
        integral = 0;
        lastError = 0;
        lastTime = tic;
    end

    currentDepth = poseData.Position.Z;
    desiredDepth = -5; % Target depth
    Kp = 1.0; Ki = 0.1; Kd = 0.05; % PID coefficients

    error = desiredDepth - currentDepth;
    currentTime = tic;
    dt = toc(lastTime);

    integral = integral + error * dt;
    derivative = (error - lastError) / dt;

    controlSignal = Kp*error + Ki*integral + Kd*derivative;

    % Send control signal to actuators
    sendControlSignal(controlSignal);

    lastError = error;
    lastTime = currentTime;
end