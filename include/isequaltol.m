function result = isequaltol(a, b, tolerance)
    % Check if the absolute difference between a and b is less than tolerance
    result = abs(a - b) < tolerance;
end