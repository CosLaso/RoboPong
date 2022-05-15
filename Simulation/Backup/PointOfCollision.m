%% Q9: Collision detection line (harder)

mdl_planar3;

[v,f,fn] = RectangularPrism([2,-1.1,-1], [3,1.1,1]);
steps = 50;
q1 = [pi/3,0,0]; 
q2 = [-pi/3,0,0];

hold on
qMatrix = jtraj(q1,q2,steps);
for i = 1:steps
    result = IsCollision(p3,qMatrix(i,:),f,v,fn);
    if result == 1
        qMatrix(i,:)
        p3.plot(q1);
        pause(3);
        p3.animate(qMatrix(1:i,:));
        break
    end 
end 