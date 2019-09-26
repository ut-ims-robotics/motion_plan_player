BEGIN{
FS = ",";
OFS = ",";
}

{
  values = sin($1/100*3.14-1.57)/2.0+0.5
  print values, 0.0, 0.0, -values, 0.0, values*2, 0.0 
}
