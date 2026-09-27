import os

path = r'c:\Users\Muhammad Zidane A\Documents\PTN\HIMTIKA\Lomba CNC\Code and all\Esp32\Esp32.ino'
with open(path, 'r', encoding='utf-8') as f:
    lines = f.readlines()

new_lines = []
skip = False
for i, line in enumerate(lines):
    if 'const char DASHBOARD_HTML[] PROGMEM =' in line:
        new_lines.append('// DASHBOARD_HTML dinonaktifkan untuk menghemat memori\n')
        new_lines.append('const char DASHBOARD_HTML[] PROGMEM = "";\n')
        skip = True
    if skip and '</html>\\n";' in line:
        skip = False
        continue
    
    if not skip:
        if 'server.on("/", HTTP_GET' in line:
            new_lines.append('  // Route Web Dashboard dinonaktifkan\n')
            new_lines.append('  // ' + line)
            if i + 1 < len(lines):
                lines[i+1] = '// ' + lines[i+1]
            if i + 2 < len(lines):
                lines[i+2] = '// ' + lines[i+2]
        else:
            new_lines.append(line)

with open(path, 'w', encoding='utf-8') as f:
    f.writelines(new_lines)
print('Berhasil dihapus!')
