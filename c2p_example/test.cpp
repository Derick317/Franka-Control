#include <iostream>
#include <cstring>
using namespace std;

int main()
{
    string s;
    cin >> s;
    cout << (s == "123") << endl;
    cin.ignore();
    cout << "OK" << endl;
    return 0;
}