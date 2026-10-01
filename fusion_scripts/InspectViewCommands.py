"""List available Fusion commands related to view orientation."""
import adsk.core
import json


def run(_context):
    ui = adsk.core.Application.get().userInterface
    commands = []
    for command in ui.commandDefinitions:
        label = ' '.join(str(x or '') for x in (command.id, command.name))
        if any(term in label.lower() for term in (
                'viewcube', 'set current view', 'set top', 'top view',
                'orientation', 'view cube')):
            commands.append({
                'id': command.id,
                'name': command.name,
            })
    print(json.dumps(commands[:100]))
