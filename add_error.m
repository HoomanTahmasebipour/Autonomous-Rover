function [value_noisy] = add_error(value, pct_error, bounds)
%ADD_ERROR Adds normally distributed percent error to a measurement
%   As an input, this function takes a measurement value and an error
%   percentage (from 0 to 1). It uses randn to calculate a normally
%   distributed error and add it to the value and output it.
%   
%   bounds is an optional two-value vector that can be added to specify
%   limits to the returned values. For example, if bounds is [0 1], values
%   will be limited to those within the given values
%   
%   Copyright (c) 2020, Ian G. Bennett
%   All rights reserved.
%   Development funded by the University of Toronto, Department of
%   Mechanical and Industrial Engineering.
%   Distributed under GNU AGPLv3 license.



err = randn * pct_error * value;
value_noisy = value + err;

if exist('bounds','var')
    if bounds(1) < bounds(2)
        if value_noisy < bounds(1)
            value_noisy = bounds(1);
        elseif value_noisy > bounds(2)
            value_noisy = bounds(2);
        end
    else
        error('Error bounding defined incorrectly, ensure lower and upper limits correct.')
    end
end

end

