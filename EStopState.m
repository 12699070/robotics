function EStopState(type)

persistent value

if isempty(value)
    value = true;
end

if value == true
    switch type
        case 1  % Press buttom
            disp('E-STOP PRESSED. ROBOT HAS BEEN PAUSED')
            
        case 2  % Collision detected
            disp('COLLISION DETECTED')
        
        case 3  % Workspace limits
            disp('ENTERING WORKSPACE')
            
    end
    value = false;
    uiwait;
        
elseif (value == false) && (type == 1)
    disp('RESUME')
    value = true;
    uiresume;
end

end
