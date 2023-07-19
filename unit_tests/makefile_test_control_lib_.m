%% still missing topics:
%- reset functions
%- destroy functions
%- verify if all the unit tests file are paranetrized 
% correctly in function of n_y,n_u, order...

%% library paths
lib_path = 'C:\projects\nw-libraries\lib-lti-systems\src\';
lib_path_lib_lti = [lib_path,'lti_systems.c'];
lib_path_lib_control = [lib_path,'control_systems.c'];
lib_path_lib_lin_algebra = [lib_path,'lin_algebra.c'];

%% lin_algerbra.h
disp('building s_test_control_lib_mat_vect_mult.c ...');
mex('s_test_control_lib_mat_vect_mult.c', lib_path_lib_lti, lib_path_lib_control, lib_path_lib_lin_algebra);

disp('building s_test_control_lib_vect_sum.c ...');
mex('s_test_control_lib_vect_sum.c', lib_path_lib_lti, lib_path_lib_control, lib_path_lib_lin_algebra);

disp('building s_test_control_lib_vect_sign_change.c ...');
mex('s_test_control_lib_vect_sign_change.c', lib_path_lib_lti, lib_path_lib_control, lib_path_lib_lin_algebra);

disp('building s_test_control_lib_zero_eye.c ...');
mex('s_test_control_lib_zero_eye.c', lib_path_lib_lti, lib_path_lib_control, lib_path_lib_lin_algebra);

%% lti_system.h
disp('building s_test_control_lib_tf.c ...');
mex('s_test_control_lib_tf.c', lib_path_lib_lti, lib_path_lib_control, lib_path_lib_lin_algebra);

disp('building s_test_control_lib_ss.c ...');
mex('s_test_control_lib_ss.c', lib_path_lib_lti, lib_path_lib_control, lib_path_lib_lin_algebra);

% control_system.h
disp('building s_test_control_lib_poly_ctr.c ...');
mex('s_test_control_lib_poly_ctr.c', lib_path_lib_lti, lib_path_lib_control, lib_path_lib_lin_algebra);

disp('building s_test_control_lib_sf_contr.c ...');
mex('s_test_control_lib_sf_contr.c', lib_path_lib_lti, lib_path_lib_control, lib_path_lib_lin_algebra);

disp('building s_test_control_lib_observer_generic.c ...');
mex('s_test_control_lib_observer_generic.c', lib_path_lib_lti, lib_path_lib_control, lib_path_lib_lin_algebra);

disp('building s_test_control_lib_sf_contr_wfobs.c ...');
mex('s_test_control_lib_sf_contr_wfobs.c', lib_path_lib_lti, lib_path_lib_control, lib_path_lib_lin_algebra);

% 
