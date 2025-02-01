#include <iostream>

#include <tobas_string_tools/core.hpp>

using namespace std;

int main()
{
  const string title = "Title Case Example";
  const string snake = "snake_case_example";
  const string pascal = "PascalCaseExample";

  cout << "Pascal from Title: " << title << " -> " << str::pascalFromTitle(title) << endl;
  cout << "Pascal from Snake: " << snake << " -> " << str::pascalFromSnake(snake) << endl;
  cout << "Title from Snake: " << snake << " -> " << str::titleFromSnake(snake) << endl;
  cout << "Snake from Pascal: " << pascal << " -> " << str::snakeFromPascal(pascal) << endl;
  cout << "Snake from Title: " << title << " -> " << str::snakeFromTitle(title) << endl;
}
