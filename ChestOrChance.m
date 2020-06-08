function ChestOrChance(kinova,cardType)
% cardType: 1-chest; 2-chance
chestLocation = [-0.2 0.2 0.04];
chanceLocation =[0.2 -0.2 0.04];
qz = zeros(1,6);

currentQ = kinova.getpos();
if cardType == 1 %chest
    %Go to card
    guessQ = [-0.6109    0.5236   -2.2689    1.5708   -0.6981    0.1745];
    endQ = kinova.ikine(transl(chestLocation),guessQ,[1 1 1 0 0 0]);
    jointTrajectory = jtraj(currentQ,endQ,30);
    for trajStep = 1:size(jointTrajectory,1)
        q = jointTrajectory(trajStep,:);
        kinova.animate(q);
    end
    %Lift up
    currentQ = kinova.getpos();
    endQ = kinova.ikine(transl(chestLocation(1,1),chestLocation(1,2),chestLocation(1,3)+0.1),guessQ,[1 1 1 0 0 0]);
    jointTrajectory = jtraj(currentQ,endQ,30);
    for trajStep = 1:size(jointTrajectory,1)
        q = jointTrajectory(trajStep,:);
        kinova.animate(q);
    end
    % Show to the couch
    currentQ = kinova.getpos();
    showQ = [1.5708    0.4363   -2.2689    1.5708    1.0472         0];
    jointTrajectory = jtraj(currentQ,showQ,30);
    for trajStep = 1:size(jointTrajectory,1)
        q = jointTrajectory(trajStep,:);
        kinova.animate(q);
    end    
    % Put the card back
    currentQ = kinova.getpos();
    guessQ = [-0.6109    0.5236   -2.2689    1.5708   -0.6981    0.1745];
    endQ = kinova.ikine(transl(chestLocation),guessQ,[1 1 1 0 0 0]);
    jointTrajectory = jtraj(currentQ,endQ,30);
    for trajStep = 1:size(jointTrajectory,1)
        q = jointTrajectory(trajStep,:);
        kinova.animate(q);
    end
    % Go home
    currentQ = kinova.getpos();
    endQ = qz;
    jointTrajectory = jtraj(currentQ,endQ,30);
    for trajStep = 1:size(jointTrajectory,1)
        q = jointTrajectory(trajStep,:);
        kinova.animate(q);
    end
end

if cardType == 2 %chance
    %Go to card
    guessQ = [-0.5236   -0.5236    2.2689   -1.5708   -0.6981    0.2618];
    endQ = kinova.ikine(transl(chanceLocation),guessQ,[1 1 1 0 0 0]);
    jointTrajectory = jtraj(currentQ,endQ,30);
    for trajStep = 1:size(jointTrajectory,1)
        q = jointTrajectory(trajStep,:);
        kinova.animate(q);
    end
    %Lift up
    currentQ = kinova.getpos();
    endQ = kinova.ikine(transl(chanceLocation(1,1),chanceLocation(1,2),chanceLocation(1,3)+0.1),guessQ,[1 1 1 0 0 0]);
    jointTrajectory = jtraj(currentQ,endQ,30);
    for trajStep = 1:size(jointTrajectory,1)
        q = jointTrajectory(trajStep,:);
        kinova.animate(q);
    end
    % Show to the couch
    currentQ = kinova.getpos();
    showQ = [-1.5708   -0.5236    2.2689   -1.5708    1.2217         0];
    jointTrajectory = jtraj(currentQ,showQ,30);
    for trajStep = 1:size(jointTrajectory,1)
        q = jointTrajectory(trajStep,:);
        kinova.animate(q);
    end    
    % Put the card back
    currentQ = kinova.getpos();
    guessQ = [-0.6109    0.5236   -2.2689    -1.5708   -0.6981    0.1745];
    endQ = kinova.ikine(transl(chanceLocation),guessQ,[1 1 1 0 0 0]);
    jointTrajectory = jtraj(currentQ,endQ,30);
    for trajStep = 1:size(jointTrajectory,1)
        q = jointTrajectory(trajStep,:);
        kinova.animate(q);
    end
    %Go home
    currentQ = kinova.getpos();
    endQ = qz;
    jointTrajectory = jtraj(currentQ,endQ,30);
    for trajStep = 1:size(jointTrajectory,1)
        q = jointTrajectory(trajStep,:);
        kinova.animate(q);
    end
end