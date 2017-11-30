function [ desired_state ] = traj_generator2(t, state, waypoints)

persistent waypoints0 traj_time d0 coffx coffy coffz
if nargin > 2
    d = waypoints(:,2:end) - waypoints(:,1:end-1);
    d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
    traj_time = [0, cumsum(d0)];
    waypoints0 = waypoints;
    
    xway = waypoints(1,:)';
    yway = waypoints(2,:)';
    zway = waypoints(3,:)';
    coffx = getCoeff(xway);
    coffy = getCoeff(yway);
    coffz = getCoeff(zway);
    
else
    if(t > traj_time(end))
        t = traj_time(end);
    end
    t_index = find(traj_time >= t,1);

    if(t_index > 1)
        t = t - traj_time(t_index-1);
        
    end
    if(t == 0)
            
        desired_state.pos = waypoints0(:,1);
    else
         scale = t/d0(t_index-1);
        desired_state.pos = (1 - scale) * waypoints0(:,t_index-1) + scale * waypoints0(:,t_index);
        desired_state.vel = (1 - scale)^2 * waypoints0(:,t_index - 1) + scale^2 * waypoints0(:,t_index);
             
%               scale = (t-traj_time(t_index)) / d0(t_index);
%               index = (t_index-1) + 1:t_index; 
%               desired_state.pos = [coffx(index)'; coffy(index)'; coffz(index)' ];
%               desired_state.vel=[coffx(index)'*(1/d0(t_index));coffy(index)'*(1/d0(t_index));coffz(index)'*(1/d0(t_index)) ];
%               desired_state.acc=[coffx(index)'*(1/d0(t_index)^2);coffy(index)'*(1/d0(t_index)^2);coffz(index)'*(1/d0(t_index)^2) ];
        
    end
    
    desired_state.vel = zeros(3,1);
    desired_state.acc = zeros(3,1);
    desired_state.yaw = 0;
    desired_state.yawdot = 0;
end
end

