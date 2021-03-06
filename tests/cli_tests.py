import subprocess

test_inputs = [
    ["./3DCopy", "one.pcd", "two.pcd", "three.pcd", "output.pcd"],
    ["./3DCopy", "one.pcd", "three.pcd", "output"],
    ["./3DCopy", "-d", "pcd_files", "output"],
    ["./3DCopy", "-d", "asg", "output"],
    ["./3DCopy", "-d", "no_pcd_files", "output"],
    ["./3DCopy", "one.txt", "two.pcd", "three.pcd", "output"],
    ["./3DCopy", "one.pcd", "output.txt"]
    ]

for test_input in test_inputs:
    p = subprocess.Popen(test_input, stdout=subprocess.PIPE)
    print(p.communicate()[0].decode())
