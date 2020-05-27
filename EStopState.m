function EStopState(value,type,inputs)

switch type
    case 1  % Press buttom
        if value == true
            disp('E-STOP PRESSED. ROBOT HAS BEEN PAUSED')
            uiwait;
        end
        if value == false
            disp('E-STOP UNPRESSED. RESUMING')
            uiresume;
        end
        
    case 2  % Detect collision
        if value == true
            disp('COLLISION DETECTED')
            pause;
        end
%         if value == false
%             disp('')
            
%         result = true;
%         while result == true
%             if EStop
%             result = IsCollision(inputs.kinova,inputs.qMatrix,inputs.faces,inputs.vertex,inputs.faceNormals,inputs.returnOnceFound);
%         end
%         disp('COLLISION ')
%         uiresume;
end

end
