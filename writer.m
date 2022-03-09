%% SCRIPT USED TO WRITE MATLAB DATA TO CSV FOR GAZEBO
% author: Pete Lealiiee Jr
start_height = 3;
writematrix([R.tVec',R.rIstar + [0 0 start_height],R.vIstar,R.aIstar,R.xIstar], 'Ali_traj1.xls' )
writematrix([R.tVec',R.rIstar + [0 0 start_height], P11_ext2d, P12_ext2d, P21_ext2d, P22_ext2d,], 'Ali_traj1_cov.xls' )
