
new_lines = []
with open('rocket.obj','r') as file:

    lines = file.readlines()

    for line in lines:

        if line[0] == '#':
            continue

        if line[0] == 'f':
            new_lines.append(line)
            continue

        row = line.split()

        x = float(row[1])*0.001
        y = float(row[2])*0.001
        z = float(row[3])*0.001 - 0.5

        string = ' {:.6f} {:.6f} {:.6f}\n'.format(x,y,z)

        new_lines.append(row[0]+ string)

with open('rocket3.obj','w') as file:
    file.writelines(new_lines)
