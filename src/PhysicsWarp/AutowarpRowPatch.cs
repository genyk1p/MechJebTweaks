using System;
using System.Collections.Generic;
using System.Reflection;
using HarmonyLib;
using MuMech;
using UnityEngine;

namespace MechJebTweaks.PhysicsWarp
{
    /// <summary>
    /// Puts the "Auto physics warp: [off][x2][x3][x4]" row right under MechJeb's own "Auto-warp" checkbox in the
    /// maneuver planner and the node editor: a transpiler inserts a call to <see cref="AutoPhysicsWarp.DrawRow"/>
    /// after the first GUILayout.EndHorizontal() that follows the store to MechJebModuleNodeExecutor.Autowarp.
    /// If MechJeb changed and that spot is not found, the window is left as is (the row stays in the
    /// "Rendezvous with offset" window).
    /// </summary>
    internal static class AutowarpRowPatch
    {
        private const string HarmonyId = "MechJebTweaks.AutoPhysicsWarp";

        private static string _patching;

        public static void Apply()
        {
            var harmony = new Harmony(HarmonyId);
            Patch(harmony, typeof(MechJebModuleManeuverPlanner));
            Patch(harmony, typeof(MechJebModuleNodeEditor));
        }

        private static void Patch(Harmony harmony, Type window)
        {
            MethodInfo gui = AccessTools.Method(window, "WindowGUI", new[] { typeof(int) });
            if (gui == null)
            {
                Log.Warning($"auto physics warp: {window.Name}.WindowGUI(int) not found, no row in that window");
                return;
            }
            try
            {
                _patching = window.Name;
                harmony.Patch(gui, transpiler: new HarmonyMethod(typeof(AutowarpRowPatch), nameof(Transpiler)));
            }
            catch (Exception e)
            {
                Log.Warning($"auto physics warp: patching {window.Name} failed, no row in that window: {e.Message}");
            }
        }

        private static IEnumerable<CodeInstruction> Transpiler(IEnumerable<CodeInstruction> instructions)
        {
            FieldInfo autowarp = AccessTools.Field(typeof(MechJebModuleNodeExecutor), nameof(MechJebModuleNodeExecutor.Autowarp));
            MethodInfo endHorizontal = AccessTools.Method(typeof(GUILayout), nameof(GUILayout.EndHorizontal));
            MethodInfo drawRow = AccessTools.Method(typeof(AutoPhysicsWarp), nameof(AutoPhysicsWarp.DrawRow));

            var list = new List<CodeInstruction>(instructions);
            int store = list.FindIndex(i => i.StoresField(autowarp));
            int end = store < 0 ? -1 : list.FindIndex(store, i => i.Calls(endHorizontal));
            if (end < 0)
            {
                Log.Warning($"auto physics warp: Auto-warp checkbox not found in {_patching}, no row there");
                return list;
            }
            list.Insert(end + 1, new CodeInstruction(System.Reflection.Emit.OpCodes.Call, drawRow));
            Log.Info($"auto physics warp: row added under Auto-warp in {_patching}");
            return list;
        }
    }
}
