function CartesianControl(kinova,x_value,y_value,z_value)

currentQ = kinova.getpos();
endQ = kinova.ikine(transl([x_value y_value z_value]),zeros(1,6),[1 1 1 0 0 0]);

showSteps = false; % true = show steps; false = plot straight away

if showSteps == false
    kinova.animate(endQ);
end

if showSteps == true
    jointTrajectory = jtraj(currentQ,endQ,30);
    for trajStep = 1:size(jointTrajectory,1)
        q = jointTrajectory(trajStep,:);
        kinova.animate(q);
    end
end

end