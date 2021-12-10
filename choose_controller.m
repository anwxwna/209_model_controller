%function that chooses the right controller for a specific model and task
%combination
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%for driving the unicycle/holonomic robot to a desired state specify 
%task_details = [[x_des y_des theta_des]',[x_init y_init theta_int]',[t t t]']
%carlike/type 21
%task_details = [[x_des y_des theta_des phi_des]',[x_init y_init theta_int phi_init]',[t t t t]']
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%for path following unicycle circle with radius R only
%task_details = [[1/R 0 0 0 0 0]',[s_init d_init theta_e_init x_init y_init theta_init]',[t t t t t t]']
%for path following carlike circle with radius R only
%task_details = [[1/R 0 0 0 0 0 0 0]',[s_init d_init theta_e_init phi_init x_init y_init theta_init phi_init ]',[t t t t t t t t]']
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%for trajectopry tracking unicycle/holonomic
%task_details = [[0 0 0 0 0 0]',[x_r_init y_r_init theta_r_init x_init y_init theta_init]',[t t t t t t]']
%for trajectopry tracking carlike
%task_details =[[0 0 0 0 0 0 0 0]',[x_r_init y_r_init theta_r_init phi_r_init x_init y_init theta_init phi_init]',[t t t t t t t t]']

function choose_controller(type_model, type_task, task_details)
    switch type_model
        case '(3,0)'
            switch type_task
                    case 'point'
                        holonomic_point(task_details)
                    case 'path'
                        holonomic_path(task_details)%not implmeneted yet
                    case 'trajectory'
                        holonomic_trajectory(task_details)
            end
        case '(2,0)'
            switch type_task
                    case 'point'
                        unicylce_point_expon(task_details)
                    case 'path'
                        unicycle_path_following(task_details)
                    case 'trajectory'
                        unicycle__trajectory_tracking(task_details)
            end
        case '(2,1)'
            switch type_task
                    case 'point'
                        type2_1point_expon(task_details)
                    case 'path'
                        type21_path(task_details)%not implemented yet
                    case 'trajectory'
                        type21_trajectory(task_details)%not implmented yet
            end
        case '(1,1)'
            switch type_task
                    case 'point'
                        carlike_point_expon(task_details)
                    case 'path'
                        car_pathfollowing(task_details)
                    case 'trajectory'
                        car_trajectory_tracking(task_details)
            end
        case '(1,2)'
            switch type_task
                    case 'point'
                        type1_2_point_expon(task_details)%not implmented yet
                    case 'path'
                        type12_path(task_details)%not implemented yet
                    case 'trajectory'
                        type12_trajectory(task_details)%not implemented yet
            end
    end
end