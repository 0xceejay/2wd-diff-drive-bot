import re
import os
import shutil

MESHES_DEST = '../meshes/'
URDF_DEST = '../urdf/bot.urdf'
ASSETS_SRC = 'assets/'

# READ THE URDF CONTENT
with open('robot.urdf', 'r') as f:
    content = f.read()

#  Remove collisions for internal/small parts
parts_to_strip = [
    'spacer', 'tt_motor_3777_v10', 'motor_holders', 'main_half_size_board',
    'half_size_side_rail', 'remache_v1_0', 'contacto_plano_v1_0',
    'porta_baterias_2x18650', 'contacto_helice_v2_0', 'bateria_18650',
    'caster_40mm_swivel_plate_01'
]

for part in parts_to_strip:
    pattern = rf'(<!-- Part {part}(?:_\d+)? -->.*?<visual>.*?</visual>\s+)<collision>.*?</collision>'
    content = re.sub(pattern, r'\1', content, flags=re.DOTALL)

# Replace specific link collisions with collision shapes
replacements = {
    'chassis': '<box size="0.19 0.10 0.04"/>',
    'wheel': '<cylinder radius="0.033" length="0.026"/>',
    'wheel_2': '<cylinder radius="0.033" length="0.026"/>',
    'caster_40mm_wheel_support_01': '<box size="0.04 0.04 0.04"/>',
    'caster_40mm_wheel_30mm_01': '<sphere radius="0.02"/>',
}

for link_name, primitive in replacements.items():
    # Targets the <link> block specifically to avoid accidental mesh replacements in <visual>
    pattern = rf'(<link name="{link_name}">.*?<collision>.*?<geometry>).*?(</geometry>)'
    content = re.sub(pattern, rf'\1\n        {primitive}\n      \2', content, flags=re.DOTALL)

# Change meshes lookup directory
content = content.replace('package://bot_description/assets/', 'package://bot_description/meshes/')
content = re.sub(r'\n\s*\n', '\n', content)

os.makedirs(os.path.dirname(URDF_DEST), exist_ok=True)
with open(URDF_DEST, 'w') as f:
    f.write(content)
print(f"✅ URDF processed and moved to {URDF_DEST}")

# MOVE MESHES (.stl files)
if not os.path.exists(MESHES_DEST):
    os.makedirs(MESHES_DEST)

if os.path.exists(ASSETS_SRC):
    files_moved = 0
    for filename in os.listdir(ASSETS_SRC):
        if filename.endswith('.stl'):
            shutil.copy(os.path.join(ASSETS_SRC, filename), os.path.join(MESHES_DEST, filename))
            files_moved += 1
    print(f"✅ {files_moved} meshes copied to: {MESHES_DEST}")


os.remove('robot.urdf')
shutil.rmtree(ASSETS_SRC)
