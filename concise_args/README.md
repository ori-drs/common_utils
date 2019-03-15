Are you frustrated with the amount of code that you need to write to parse command line arguments? Well then concise-args is for you!

It is a simple command line argument parser that allows very concise syntax.

There is only a single header, with no dependancies other than the normal STL containers.

The goal was to require as few lines of code as possible to parse options passed in using the standard -c/--longName= syntax.

For example, to parse 4 flags requires 6 lines of code:
```
#include <ConciseArgs>
int main(int argc, char ** argv) {
  bool bl = false;  int in;  float flt = -9; double dbl=3.14;
  ConciseArgs parser(argc, argv);
  parser.add(bl,  "b", "bools", "do bools work?");       // Parse -b/--bools, setting bl accordingly
  parser.add(in,  "i", "ints",  "do ints work?", true);  // The "true" means this argument is mandatory
  parser.add(flt, "f", "floats");                        //I'm too lazy to provide a description
  parser.add(dbl, "d");                                  //I'm too lazy to even provide a longName
  parser.parse();
  return 0;
}
```
This would set the create the following help/usage message:

```
$ ./concise-args-test -h
Usage:
  concise-args-test [opts] 
Options:
  -h, --help   = [true]       : Display this help message
  -b, --bools  = [false]      : do bools work?
  -i, --ints   = <int32_t>    : do ints work?
  -f, --floats = [-9.00000]   : 
  -d           = [3.14000]    : 
```

==============================================================================
This software is constructed according to the Pods software policies and
templates.  The policies and templates can be found at:

  http://sourceforge.net/projects/pods

===============================================================================
License:      The MIT License (MIT)
Copyright (c) <2012> <Abraham Bachrach>

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
