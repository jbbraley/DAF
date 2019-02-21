function new = clone(self,name)
%% clone() -> create new instance of object and populate public properties
%
% * auto-fills public properties
% * supply a parent object handle to spawn object other than self
%   * useful when creating inherited properties
%
% author: john devitis; john braley
% create date: 21-Nov-2016 11:01:41
    if nargin > 1
        new = feval(class(name));
    else
        new = feval(class(self));
    end
    p = properties(self);
    info = metaclass(self);
    for ii = 1:length(p)
        if ~info.PropertyList(ii).Dependent
            new.(p{ii}) = self.(p{ii});
        end
    end
end
