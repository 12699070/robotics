function eStopState(value)
if value == true
    disp('E-STOP PRESSED. ROBOT HAS BEEN PAUSED')
    uiwait;
end

if value == false
    disp('E-STOP UNPRESSED. RESUMING')
    uiresume;
end
end
