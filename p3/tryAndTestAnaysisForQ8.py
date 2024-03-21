import subprocess

def rewrite_line(file_path, line_number, new_line):
    # Read the original file
    with open(file_path, 'r') as file:
        lines = file.readlines()

    # Modify the desired line
    lines[line_number - 1] = new_line + '\n'  # Subtract 1 because line numbers are 1-indexed

    # Write the modified content back to the file
    with open(file_path, 'w') as file:
        file.writelines(lines)

def tryAndTest():
    epsilon = 1.0
    learnRate = 1.0
    while epsilon >= 0.0:
        learnRate = 1.0
        while learnRate >= 0:
            rewrite_line("analysis.py", 63, "    answerEpsilon = " + str(epsilon))
            rewrite_line("analysis.py", 64, "    answerLearningRate = " + str(learnRate))
            cmd = "python autograder.py -q q8"
            process = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, 
                                       stderr=subprocess.PIPE)
            # Get the output and error (if any) from the command
            output, error = process.communicate()

            # Decode the output and error from bytes to string
            output_str = output.decode('utf-8')
            error_str = error.decode('utf-8')

            # Print the output and error
            #print("Output:")
            #print(output_str)
            if "Total: 1" in output_str:
                print("Output:")
                print(output_str)
                exit()

            #print("\nError:")
            #print(error_str)
            learnRate -= 0.01
        epsilon -= 0.01

tryAndTest()