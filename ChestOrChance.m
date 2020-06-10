function ChestOrChance(kinova,cardType,card)
    % cardType: 1-chest; 2-chance
    qz = zeros(1,6);

    currentQ = kinova.getpos();

    if cardType == 1 % Chest
        guessQ1 = [-0.6109    0.5236   -2.2689    1.5708   -0.6981    0.1745];
        showQ = [1.5708    0.4363   -2.2689    1.5708    1.0472         0];
        guessQ2 = [-0.6109    0.5236   -2.2689    1.5708   -0.6981    0.1745];
        location = [-0.2 0.2 0.04]; % Chest location

    elseif cardType == 2 % Chance
        guessQ1 = [-0.5236   -0.5236    2.2689   -1.5708   -0.6981    0.2618];
        showQ = [-1.5708   -0.5236    2.2689   -1.5708    1.2217         0];
        guessQ2 = [-0.6109    0.5236   -2.2689    -1.5708   -0.6981    0.1745];   
        location =[0.2 -0.2 0.04];  % Chance location
    end


    % Movement

    %Go to card
    endQ = kinova.ikine(transl(location),guessQ1,[1 1 1 0 0 0]);
    jointTrajectory = jtraj(currentQ,endQ,30);
    for trajStep = 1:size(jointTrajectory,1)
        q = jointTrajectory(trajStep,:);
        kinova.animate(q);
    end

    %Lift up
    currentQ = kinova.getpos();
    card.relation = inv(kinova.fkine(currentQ)) * card.figTr; % Transform between eff and card
    endQ = kinova.ikine(transl(location(1,1),location(1,2),location(1,3)+0.1),guessQ1,[1 1 1 0 0 0]);
    jointTrajectory = jtraj(currentQ,endQ,30);
    MovementCard(jointTrajectory,kinova,card);

    % Show to the couch
    currentQ = kinova.getpos();
    jointTrajectory = jtraj(currentQ,showQ,30);
    MovementCard(jointTrajectory,kinova,card);

    % Put the card back
    currentQ = kinova.getpos();
    endQ = kinova.ikine(transl(location),guessQ2,[1 1 1 0 0 0]);
    jointTrajectory = jtraj(currentQ,endQ,30);
    MovementCard(jointTrajectory,kinova,card);

    % Go home
    currentQ = kinova.getpos();
    endQ = qz;
    jointTrajectory = jtraj(currentQ,endQ,30);
    for trajStep = 1:size(jointTrajectory,1)
        q = jointTrajectory(trajStep,:);
        kinova.animate(q);
    end

end
%% Movement of the card
function MovementCard(jointTrajectory,kinova,card)
    for trajStep = 1:size(jointTrajectory,1)
        q = jointTrajectory(trajStep,:);
        kinova.animate(q);
        
        % Update piece location
        card.pose = kinova.fkine(q) * card.relation;
        card.updatedPoints = (card.pose * [card.verts,ones(card.vertexCount,1)]')';     % Transform the vertices
        card.mesh_h.Vertices = card.updatedPoints(:,1:3);                             % Update the mesh vertices in the patch handle
    end
end