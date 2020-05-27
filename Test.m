h_fig = figure(1);
set(h_fig,'KeyPressFcn',@func);

function func(src,event)

persistent x
persistent y
persistent z

if isempty(x)
    x = 0;
end
if isempty(y)
    y = 0;
end
if isempty(z)
    z = 0;
end

disp(event.Key);

switch(event.Key)
    case 'uparrow'
        x = x + mov;
    case 'downarrow'
        x = x - mov;
    case 'rightarrow'
        y = y + mov;
    case 'leftarrow'
        y = y - mov; 
    case 't'
        z = z + mov;
    case 'g'
        z = z - mov;
end
brick.pos = [x,y,z]

end