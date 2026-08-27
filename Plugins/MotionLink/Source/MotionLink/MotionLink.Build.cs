using UnrealBuildTool;
using System.IO;

public class MotionLink : ModuleRules
{
	public MotionLink(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(new string[]
		{
			"Core",
			"CoreUObject",
			"Engine",
		});

		// shared/ holds the wire protocol header used verbatim by both this
		// plugin and the standalone controller_sim. ModuleDirectory is
		// .../Plugins/MotionLink/Source/MotionLink; four levels up is the repo
		// root that contains shared/.
		PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "..", "..", "..", "..", "shared"));

		if (Target.Platform == UnrealTargetPlatform.Win64)
		{
			// Raw Winsock UDP + high-resolution waitable timer live in the
			// worker; link the platform libraries directly.
			PublicSystemLibraries.Add("ws2_32.lib");
			PublicSystemLibraries.Add("winmm.lib");
		}
	}
}
