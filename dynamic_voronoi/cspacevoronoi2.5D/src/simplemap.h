#ifndef SIMPLEMAP_H_
#define SIMPLEMAP_H_

#include <stdio.h>
#include <iostream>
#include <assert.h>
#include <string.h>

template <class T>
class SimpleMap {
public:
  SimpleMap() {
    sizeX = -1;
    sizeY = -1;
    data = NULL;
  }
  SimpleMap(int w, int h) {
    data = NULL;
    sizeX = -1;
    sizeY = -1;
    resize(w,h);
  }
  ~SimpleMap() {
    if (data) {
      for (int x=0; x<sizeX; x++) {
        delete[] data[x];
      }
      delete[] data;
    }
  }
  int getMapSizeX() { return sizeX; }
  int getMapSizeY() { return sizeY; }

  
  inline T preIncrement(int x, int y) {
    return ++data[x][y];
  }
  inline T preDecrement(int x, int y) {
    return --data[x][y];
  }


  inline T getCell(int x, int y) { return data[x][y]; }
  inline T getCell(INTPOINT p) { return data[p.x][p.y]; }
  inline void setCell(int x, int y, T val) { data[x][y] = val; }

  void resize(int width, int height) {
    if (data && (width!=sizeX || height!=sizeY)) {
      for (int x=0; x<sizeX; x++) {
        delete[] data[x];
      }
      delete[] data;
      data = NULL;
    }

    if (!data) {
      data = new T*[width];
      sizeX = width;
      sizeY = height;
      for (int x=0; x<sizeX; x++) {
        data[x] = new T[height];
      }
    }
  }

  void fill(T val) {
    if (!data) {
      fprintf(stderr, "SimpleImage not initialized (fill)\n");
      return;
    }
    for (int x=0; x<sizeX; x++) {
      for (int y=0; y<sizeY; y++) {
        data[x][y] = val;
      }
    }    
  }

  void writeBinary(FILE* f) const {
    fwrite( (const char*)&sizeX, sizeof(int), 1, f);
    fwrite( (const char*)&sizeY, sizeof(int), 1, f);

    for (int x=0; x<sizeX; x++) {
      for (int y=0; y<sizeY; y++) {
        float v = (float)data[x][y];
        fwrite( (const char*)&v, sizeof(float), 1, f);
      }
    }
    fflush(f);
  }
  void readBinary(std::istream &s) {
    
    int newSizeX = -99;
    int newSizeY = -88;
    s.read((char*)&newSizeX, sizeof(int));
    s.read((char*)&newSizeY, sizeof(int));
    //    printf("got values %d %d\n", newSizeX, newSizeY);
    assert(newSizeX>0 && newSizeY>0);
    resize(newSizeX, newSizeY);
    for (int x=0; x<sizeX; x++) {
      s.read((char*)data[x], sizeof(T)*sizeY);
    }
  }

  void copyFrom(SimpleMap* old) {
    resize(old->getMapSizeX(), old->getMapSizeY());
    for (int x=0; x<sizeX; x++) memcpy(data[x], old->data[x], sizeof(T)*sizeY);
  }


  void writeToPGM(const char* filename, bool normalize = true){
    
    T max = 1;
    if(normalize){
      for(int x = 0; x<sizeX; x++){
        for(int y = 0; y<sizeY; y++){      
          T val = data[x][y];
          if(val > max)
            max = val;
        }
      }
    }
    FILE* F = fopen(filename, "w");
    if (!F) {
      printf("could not open file \"%s\" for writing!\n", filename);
      return;
    }
          
    fprintf(F, "P5\n#\n");
    fprintf(F, "%d %d\n255\n", sizeX, sizeY);
    //    for(int y = 0; y<sizeY; y++){      
    for(int y = sizeY-1; y >=0; y--){      
      for(int x = 0; x<sizeX; x++){	
        float f = 255*(double) data[x][y]/ (double) max;
        if(f > 255)
          f=255;
        char c;
        if   (f>=0) c=(unsigned char) f;
        else        c= 127+f;
        fputc( c, F );
      }
    }
    
    fclose(F);
  }

  void loadFromPGM( std::istream &is ) {
    std::string tag;
    is >> tag;
    if (tag!="P5") {
      std::cerr << "Awaiting 'P5' in pgm header, found " << tag << std::endl;
      exit(-1);
    }
  
    int newSizeX, newSizeY;

    while (is.peek()==' ' || is.peek()=='\n') is.ignore();
    while (is.peek()=='#') is.ignore(255, '\n');
    is >> newSizeX;
    while (is.peek()=='#') is.ignore(255, '\n');
    is >> newSizeY;
    while (is.peek()=='#') is.ignore(255, '\n');
    is >> tag;
    if (tag!="255") {
      std::cerr << "Awaiting '255' in pgm header, found " << tag << std::endl;
      exit(-1);
    }

    resize(newSizeX, newSizeY);

    for (int y=sizeY-1; y>=0; y--) {
      for (int x=0; x<sizeX; x++) {
        int c = is.get();
        if ((double)c<255-255*0.2) setCell(x,y,1); // cell is occupied
        else setCell(x,y,0); // cell is free
        if (!is.good()) {
          std::cerr << "Error reading pgm map.\n";
          exit(-1);
        }
      }
    }
  }


  T **data;

private:
  int sizeX;
  int sizeY;
};


#endif
