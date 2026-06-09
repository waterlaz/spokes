#pragma once

#include "math.h"
#include <string>
#include <sstream>
#include <fstream>

class PostScript {
private:
    std::stringstream code;
    float x1 = 0;
    float y1 = 0;
    float x2 = 210;
    float y2 = 297;
    const char* arrowHeadDef =
        "/arrowhead {% stack: s x1 y1, current point: x0 y0\n"
        "\tgsave\n"
        "\tcurrentpoint % s x1 y1 x0 y0\n"
        "\t4 2 roll exch % s x0 y0 y1 x1\n"
        "\t4 -1 roll exch % s y0 y1 x0 x1\n"
        "\tsub 3 1 roll % s x1-x2 y0 y1\n"
        "\tsub exch % s y0-y1 x1-x2\n"
        "\tatan rotate % rotate over arctan((y0-y1)/(x1-x2))\n"
        "\tdup scale % scale by factor s\n"
        "\t-7 2 rlineto 1 -2 rlineto -1 -2 rlineto\n"
        "\tclosepath fill % fill arrowhead\n"
        "\tgrestore\n"
        "\tnewpath\n"
        "} def\n";

public:
    PostScript() {}
    PostScript(float _x1,
               float _y1,
               float _x2,
               float _y2) :
        x1{_x1}, y1{_y1}, x2{_x2}, y2{_y2}
    {
    }
    void setFont(float size){
        code<<"/Times-Roman findfont\n"
            <<size<<" scalefont\n"
            <<"setfont\n";
    }

    void show(float x, float y, const std::string& s){
        moveto(x, y);
        code<<"("<<s<<") show\n";
    }
    void setColor(uint8_t r, uint8_t g, uint8_t b){
        code<<r/255.0<<" "<<g/255.0<<" "<<b/255.0<<" setrgbcolor\n";
    }
    void setLineWidth(float w){
        code<<w<<" setlinewidth\n";
    }
    void line(float x1, float y1, float x2, float y2){
        code<<x1<<" "<<y1<<" newpath moveto\n"
            <<x2<<" "<<y2<<" lineto\nstroke\n";
    }
    void arrow(float scale, float x1, float y1, float x2, float y2){
        line(x1, y1, x2, y2);
        arrowHead(scale, x2, y2, x1, y1);
    }
    void gsave(){
        code<<"gsave\n";
    }
    void grestore(){
        code<<"grestore\n";
    }
    void translate(float x, float y){
        code<<x<<" "<<y<<" translate\n";
    }
    void moveto(float x, float y){
        code<<x<<" "<<y<<" moveto\n";
    }
    void lineto(float x, float y){
        code<<x<<" "<<y<<" lineto\n";
    }
    void arrowHead(float scale, float x0, float y0, float x1, float y1){
        code<<x0<<" "<<y0<<" newpath moveto\n";
        code<<scale<<" "<<x1<<" "<<y1<<" arrowhead\nstroke\n";
    }
    void rotate(float a){
        code<<180*a/M_PI<<" rotate\n";
    }
    void rotateDeg(float a){
        code<<a<<" rotate\n";
    }
    void scale(float a){
        code<<a<<" "<<a<<" scale\n";
    }
    void raw(std::string s){
        code<<s;
    }
    void comment(std::string s){
        code<<"%"<<s<<"\n";
    }
    template <typename T>
    void polyLine(const std::vector<std::pair<T, T>>& p, int closed=0, int filled=0){
        code<<p[0].first<<" "<<p[0].second<<" newpath moveto\n";
        for(size_t i=1; i<p.size(); i++){
            code<<p[i].first<<" "<<p[i].second<<" lineto\n";
        }
        if(closed){
            code<<"closepath\n";
        }
        if(filled){
            code<<"gsave\nfill\ngrestore\n";
        }
        code<<"stroke\n";
    }
    template <typename T>
    void polyLine(const std::vector<std::vector<T>>& p, int closed=0, int filled=0){
        std::vector<std::pair<T, T>> pp;
        for(auto t : p){
            pp.push_back(std::pair<T, T>(t[0], t[1]));
        }
        polyLine(pp, closed, filled);
    }
    void circle(float x, float y, float r, bool filled=false){
        code<<"newpath\n";
        code<<x<<" "<<y<<" "<<r<<" 0 360 arc closepath\n";
        if(filled){
            code<<"gsave\nfill\ngrestore\n";
        }
        code<<"stroke\n";
    }
    void arc(float x, float y, float r,
             float a1, float a2, bool filled=false)
    {
        code<<x<<" "<<y<<" "<<r<<" "<<a1<<" "<<a2<<" arc\n";
        if(filled){
            code<<"gsave\nfill\ngrestore\n";
        }
        code<<"stroke\n";
    }
    float toPt(float mm){
        return mm*72/25.4;
    }
    std::string boundingBox(){
        std::stringstream s;
        s<<"%%BoundingBox: "
         <<(int)toPt(x1)<<" "
         <<(int)toPt(y1)<<" "
         <<(int)toPt(x2)<<" "
         <<(int)toPt(y2)<<"\n";
        s<<"%%HiResBoundingBox: "
         <<toPt(x1)<<" "
         <<toPt(y1)<<" "
         <<toPt(x2)<<" "
         <<toPt(y2)<<"\n";
        return s.str();
    }
    std::string toString(){
        return "%!PS-Adobe-2.0 EPSF-2.0\n"
             + boundingBox()
             + "72 25.4 div\ndup\nscale\n"
             + arrowHeadDef
             + code.str() + "showpage\n";
    }
    void saveFile(std::string path){
        std::ofstream f(path);
        f<<toString();
    }
};
