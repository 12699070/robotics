function EStopState(value,type,inputs)
if value == true
    switch type
        case 1 
            disp('E-STOP PRESSED. ROBOT HAS BEEN PAUSED')
        case 2
            disp('COLLISION DETECTED')
    end
    uiwait;
end

if value == false
    switch type
        case 1  % Press STOP button
            disp('E-STOP UNPRESSED. RESUMING')
        case 2  % Detect collision
%             result = true;
%             while result == true
%                 result = IsCollision(inputs.kinova,inputs.qMatrix,inputs.faces,inputs.vertex,inputs.faceNormals,inputs.returnOnceFound);
%             end
            disp('E-STOP UNPRESSED. RESUMING')
    end
    uiresume;
end

end
