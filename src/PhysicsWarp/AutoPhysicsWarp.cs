using System;
using System.IO;
using KSP.Localization;
using MuMech;
using UnityEngine;

namespace MechJebTweaks.PhysicsWarp
{
    /// <summary>
    /// The global "auto physics warp" choice (off / x2 / x3 / x4), shared by every vessel and remembered
    /// between games in PluginData, and the code that sets physics warp for MechJeb's node executor
    /// (<see cref="ExecutorPhysicsWarp"/>) and the safe rendezvous autopilot.
    /// </summary>
    internal static class AutoPhysicsWarp
    {
        /// <summary>The choices: 0 = off, otherwise the physics warp rate.</summary>
        public static readonly int[] Rates = { 0, 2, 3, 4 };

        private static int _rate = -1;
        private static string[] _labels;

        private static string FilePath =>
            Path.Combine(KSPUtil.ApplicationRootPath, "GameData", "MechJebTweaks", "PluginData", "AutoPhysicsWarp.cfg");

        /// <summary>The chosen rate, 0 = off.</summary>
        public static int Rate
        {
            get
            {
                if (_rate < 0)
                    Load();
                return _rate;
            }
            set
            {
                if (value == Rate)
                    return;
                _rate = value;
                Save();
                Log.Info($"auto physics warp: {(value > 1 ? $"x{value}" : "off")}");
            }
        }

        public static bool On => Rate > 1;

        private static void Load()
        {
            _rate = Settings.AutoPhysicsWarpDefaultRate;
            try
            {
                if (File.Exists(FilePath))
                {
                    ConfigNode node = ConfigNode.Load(FilePath);
                    int rate = _rate;
                    if (node != null && node.TryGetValue("Rate", ref rate))
                        _rate = rate;
                }
            }
            catch (Exception e)
            {
                Log.Warning($"auto physics warp: cannot read {FilePath}: {e.Message}");
            }
            if (Array.IndexOf(Rates, _rate) < 0)
                _rate = 0;
        }

        private static void Save()
        {
            try
            {
                Directory.CreateDirectory(Path.GetDirectoryName(FilePath));
                var node = new ConfigNode();
                node.AddValue("Rate", _rate);
                node.Save(FilePath);
            }
            catch (Exception e)
            {
                Log.Warning($"auto physics warp: cannot write {FilePath}: {e.Message}");
            }
        }

        /// <summary>
        /// With the option on: physics warp at the chosen rate when wanted, 1x otherwise. On-rails warp is
        /// never touched here. With the option off nothing is done.
        ///
        /// The mode and the rate are set in the same tick. In vacuum KSP's TimeWarp.Update switches physics mode
        /// at 1x back to on-rails every frame, and MechJeb's WarpPhysicsAtRate switches the mode in one physics tick
        /// and raises the rate only in the next one: with one physics tick per frame (high FPS) the rate never
        /// rose (2026-10-13, transpor-5m-kerbin: a 15 s turn and the Hohmann burn at 1x), at low FPS it did.
        /// </summary>
        public static void Apply(MechJebCore core, Vessel vessel, bool wanted)
        {
            if (!On)
                return;
            if (TimeWarp.WarpMode == TimeWarp.Modes.HIGH && TimeWarp.CurrentRateIndex > 0)
                return;
            if (wanted && !vessel.packed)
            {
                int index = RateIndex(Rate);
                if (TimeWarp.WarpMode == TimeWarp.Modes.LOW && TimeWarp.CurrentRateIndex == index)
                    return;
                // A finished WarpToUT stays set in MechJeb until its time passes, and MechJeb's warp
                // controller then switches back to on-rails mode every tick.
                if (core.Warp.warpToUT > 0 && TimeWarp.CurrentRateIndex == 0)
                    core.Warp.MinimumWarp();
                TimeWarp.fetch.Mode = TimeWarp.Modes.LOW;
                TimeWarp.SetRate(index, false, false);
            }
            else if (TimeWarp.CurrentRateIndex > 0)
                TimeWarp.SetRate(0, true);
        }

        /// <summary>Back to 1x if physics warp (anyone's, also the player's) is on.</summary>
        public static void DropPhysicsWarp()
        {
            if (TimeWarp.WarpMode == TimeWarp.Modes.LOW && TimeWarp.CurrentRateIndex > 0)
                TimeWarp.SetRate(0, true);
        }

        /// <summary>Highest physics warp index whose rate does not exceed the given one.</summary>
        private static int RateIndex(int rate)
        {
            float[] rates = TimeWarp.fetch.physicsWarpRates;
            int index = 0;
            while (index + 1 < rates.Length && rates[index + 1] <= rate)
                index++;
            return index;
        }

        /// <summary>One GUI row "Auto physics warp: [off][x2][x3][x4]", for MechJeb's windows and ours.</summary>
        public static void DrawRow()
        {
            try
            {
                if (_labels == null)
                {
                    _labels = new string[Rates.Length];
                    for (int i = 0; i < Rates.Length; i++)
                        _labels[i] = Rates[i] > 1 ? $"×{Rates[i]}" : Localizer.Format("#LOC_MJTweaks_APW_Off");
                }
                GUILayout.BeginHorizontal();
                GUILayout.Label(Localizer.Format("#LOC_MJTweaks_APW_Label"), GUILayout.ExpandWidth(false));
                int index = Math.Max(0, Array.IndexOf(Rates, Rate));
                int chosen = GUILayout.Toolbar(index, _labels);
                GUILayout.EndHorizontal();
                if (chosen != index && chosen >= 0 && chosen < Rates.Length)
                    Rate = Rates[chosen];
            }
            catch (Exception e)
            {
                Log.Error($"auto physics warp: drawing the row failed: {e}");
            }
        }
    }
}
