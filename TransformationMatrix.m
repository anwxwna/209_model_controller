function H = TransformationMatrix(TR)
roll_angle = TR.roll_angle;
yaw_angle  =TR.yaw_angle;
pitch_angle = TR.pitch_angle;
trans_x = TR.trans_x;
trans_y = TR.trans_y;
trans_z= TR.trans_z;

H =  [cos(yaw_angle)*cos(pitch_angle) cos(yaw_angle)*sin(pitch_angle)*sin(roll_angle)-sin(yaw_angle)*cos(roll_angle) cos(yaw_angle)*sin(pitch_angle)*cos(roll_angle)+sin(yaw_angle)*sin(roll_angle) trans_x;
      sin(yaw_angle)*cos(pitch_angle) sin(yaw_angle)*sin(pitch_angle)*sin(roll_angle)+cos(yaw_angle)*cos(roll_angle) sin(yaw_angle)*sin(pitch_angle)*cos(roll_angle)-cos(yaw_angle)*sin(roll_angle) trans_y;
      -sin(pitch_angle) cos(pitch_angle)*sin(roll_angle) cos(pitch_angle)*cos(roll_angle) trans_z;
      0     0   0  1];
end


