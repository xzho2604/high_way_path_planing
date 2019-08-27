#include <iostream>

using namespace std;

int main() {
    int a = 99;
    int b = 0;
    b = a > 0 ? 40: b;
    cout << b << endl;

    if( a > b || b == 0 ) cout << "good" << endl;

}
