function [ desired_state ] = traj_generator(t, state, waypoints)

persistent traj_time d0 coffx coffy coffz

if nargin > 2
    d = waypoints(:,2:end) - waypoints(:,1:end-1);
    d0 = 3.5 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
    traj_time = [0, cumsum(d0)];
    waypoints0 = waypoints;
    
    desired_state.vel = zeros(3,1);
    desired_state.acc = zeros(3,1);
    desired_state.yaw = 0;
    desired_state.yawdot = 0;
    
    xway = waypoints(1,:)';
    yway = waypoints(2,:)';
    zway=waypoints(3,:)';
    coffx = getCoeff(xway);
    coffy = getCoeff(yway);
    coffz = getCoeff(zway);
    
else
    if(t > traj_time(end))
        t = traj_time(end);
    end
    
    t_index = find(traj_time >= t,1)-1;

    if(t_index == 0)
        t_index = 1; %t = t - traj_time(t_index-1);
    end
    
    desired_state.yaw = 0;
    desired_state.yawdot = 0;

    scale = (t-traj_time(t_index)) / d0(t_index);
    t0 = polyT(8,0,scale)';    % t0 = polyT(8,0,scale)';
    t1 = polyT(8,1,scale)';    % t1 = polyT(8,1,scale)';
    t2 = polyT(8,2,scale)';    % t2 = polyT(8,2,scale)'; 
    index = (t_index-1)*8+1:t_index*8;  %    index = (t_index-1)*8+1:t_index*8;
    
    desired_state.pos = [coffx(index)'*t0;coffy(index)'*t0;coffz(index)'*t0 ];
    desired_state.vel=[coffx(index)'*t1.*(1/d0(t_index));coffy(index)'*t1.*(1/d0(t_index));coffz(index)'*t1.*(1/d0(t_index)) ];
    desired_state.acc=[coffx(index)'*t2.*(1/d0(t_index)^2);coffy(index)'*t2.*(1/d0(t_index)^2);coffz(index)'*t2.*(1/d0(t_index)^2) ];
    
    
end
end

