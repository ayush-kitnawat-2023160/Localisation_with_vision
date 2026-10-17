#!/usr/bin/env python3
"""Compile the production scan-context matcher in isolation; compare to pre-change reference.

No ROS, bag replay, image build, or map files required. This checks the exact
candidate distance and circular yaw shift against the historical loop and
reports a single-thread microbenchmark (NOT whole-pipeline CPU speed).
"""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / "src/step3_global_acquirer.cpp").read_text()
start = source.index("std::pair<double,int> bestDescriptorDistance(")
end = source.index("\n}\n\nclass Step3GlobalAcquirer", start)
production = source[start:end]

test = r'''
#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>
#include <utility>
#include <vector>
constexpr int kRings=20, kSectors=60;
struct Descriptor { std::array<double,kRings*kSectors> v{}; std::array<double,kRings> ring{}; };
std::pair<double,int> bestDescriptorDistance_old(const Descriptor& q,const Descriptor& m){
  double best=1e9;int shift=0;
  for(int s=0;s<kSectors;++s){
    double sim=0;int effective=0;
    for(int c=0;c<kSectors;++c){
      double dot=0,nq=0,nm=0;int mc=(c+s)%kSectors;
      for(int r=0;r<kRings;++r){
        double a=q.v[r*kSectors+c],b=m.v[r*kSectors+mc];
        dot+=a*b;nq+=a*a;nm+=b*b;
      }
      if(nq>1e-12&&nm>1e-12){sim+=dot/std::sqrt(nq*nm);++effective;}
    }
    double d=effective?1.0-sim/effective:1.0;
    if(d<best){best=d;shift=s;}
  }
  return {best,shift};
}
// PRODUCTION_FUNCTION
int main(){
  std::mt19937 rng(20261015);
  std::uniform_real_distribution<double> value(-10.,10.);
  std::vector<std::pair<Descriptor,Descriptor>> cases(400);
  for(size_t i=0;i<cases.size();++i){
    auto& q=cases[i].first;auto& m=cases[i].second;
    for(size_t j=0;j<q.v.size();++j){
      q.v[j]=(rng()%100<(i%6)*15)?0:value(rng);
      m.v[j]=(rng()%100<((i+3)%6)*15)?0:value(rng);
    }
  }
  cases[0]={Descriptor{},Descriptor{}};
  for(size_t i=1;i<10;++i){cases[i].second=cases[i].first;}
  for(size_t i=0;i<cases.size();++i){
    auto a=bestDescriptorDistance_old(cases[i].first,cases[i].second);
    auto b=bestDescriptorDistance(cases[i].first,cases[i].second);
    if(a.second!=b.second||std::abs(a.first-b.first)>1e-12){
      std::fprintf(stderr,"FAIL %zu old=(%.17g,%d) new=(%.17g,%d)\n",
          i,a.first,a.second,b.first,b.second);return 1;
    }
  }
  volatile double sink=0;
  auto bench=[&](bool newer){
    const auto begin=std::chrono::steady_clock::now();
    for(int n=0;n<10;++n){
      for(const auto& pair:cases){
        auto x=newer?bestDescriptorDistance(pair.first,pair.second):
          bestDescriptorDistance_old(pair.first,pair.second);
        sink+=x.first;
      }
    }
    return std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
  };
  const double old_ms=bench(false),new_ms=bench(true);
  std::printf("PASS: %zu randomized/edge descriptor pairs. Old %.1f ms, cached %.1f ms, speedup %.2fx (matcher only); sink=%g\n",
              cases.size(),old_ms,new_ms,old_ms/new_ms,sink);
}
'''.replace("// PRODUCTION_FUNCTION", production)
with tempfile.TemporaryDirectory(prefix="step3_scancontext_") as tmp:
    cpp = Path(tmp) / "matcher_test.cpp"
    binary = Path(tmp) / "matcher_test"
    cpp.write_text(test)
    subprocess.run(["g++", "-std=c++14", "-O3", "-Wall", "-Wextra", str(cpp), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
