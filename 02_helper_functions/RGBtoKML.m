function kml = RGBtoKML(rgb)
% Converts a rgb color code to the kml color code without intensity
% (r, g, b) -> "BBGGRR"

if numel(rgb) ~= 3
   error("RGBtoKML: The input rgb color has not 3 elements.")
end

if (max(rgb)>1.0) || (min(rgb)<0.0)
    error("RGBtoKML: The input rgb color has not values between 0 to 1.")
end

kml = strcat(dec2hex(round(rgb(3)*255),2), dec2hex(round(rgb(2)*255),2), dec2hex(round(rgb(1)*255),2));
end

