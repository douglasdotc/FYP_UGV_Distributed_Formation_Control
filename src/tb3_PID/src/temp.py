

kk = [['aa','bb','cc']]
kk.append([1,2,3])

print(kk)

dir_datalog = '/home/kwlee/catkin_ws/src/TurtleBot3_FYP2020/src/tb3_PID/datalog/yoyo.txt'

with open(dir_datalog, 'w+') as file_:
    for nested_list in kk:
        for word in nested_list:
            file_.write(str(word) + '\t')
        file_.write('\n')