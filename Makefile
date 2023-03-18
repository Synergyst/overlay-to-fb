all: overlay-to-fb

overlay-to-fb: overlay-to-fb.cpp
  g++ -O3 -pedantic -Wall -std=c++20 -lv4l2 -fopenmp $^ -o $@ -lpng

clean:
  rm -f capture
