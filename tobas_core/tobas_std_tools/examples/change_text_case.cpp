#include <iostream>

#include <tobas_std_tools/string.hpp>

using namespace std;

int main()
{
  const string title = "Title Case Example";
  const string snake = "snake_case_example";
  const string pascal = "PascalCaseExample";

  cout << "Pascal from Title: " << title << " -> " << tobas_std::pascalFromTitle(title) << endl;
  cout << "Pascal from Snake: " << snake << " -> " << tobas_std::pascalFromSnake(snake) << endl;
  cout << "Title from Snake: " << snake << " -> " << tobas_std::titleFromSnake(snake) << endl;
  cout << "Snake from Pascal: " << pascal << " -> " << tobas_std::snakeFromPascal(pascal) << endl;
  cout << "Snake from Title: " << title << " -> " << tobas_std::snakeFromTitle(title) << endl;
}
