步骤：
打开项目文件夹路径
git init
git add README.md(这一步可以在创建仓库时建立README.md)
git commit -m "first commit"
git remote add origin  git的地址
git push -u origin master



git commit -m "first commit"出现
*** Please tell me who you are.
的解决办法：
输入以下两条指令
git config --global user.email "1355134694@qq.com"
git config --global user.name "a1wj1"


入 git push -u origin master 的时,提示
fatal: 'origin' does not appear to be a git repository
fatal: Could not read from remote repository.
Please make sure you have the correct access rights
and the repository exists.

控制台中输入
git remote add origin git@git地址

再次输入
git push -u origin master
