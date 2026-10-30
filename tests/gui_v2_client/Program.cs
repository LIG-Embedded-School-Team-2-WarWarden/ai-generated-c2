using AadcCommandCenter;
using System.Text.Json;
using System.Runtime.InteropServices;
if (args.Length == 2 && args[0] == "--view-check") {
    var assembly = System.Reflection.Assembly.LoadFrom(System.IO.Path.GetFullPath(args[1]));
    var viewType = assembly.GetType("AadcCommandCenter.AssignmentView")!;
    var stateType = assembly.GetType("AadcCommandCenter.AssignmentState")!;
    var fcsType = assembly.GetType("AadcCommandCenter.FcsAsset")!;
    var targetType = assembly.GetType("AadcCommandCenter.TargetRecord")!;
    var view = Activator.CreateInstance(viewType, 1001u, 1u)!;
    var target = Activator.CreateInstance(targetType, 1001u, 131, 2, 3.01f, -.2f, 3f, 3f, 1u, (byte)0, true, 1u, true)!;
    viewType.GetMethod("SetCommandStatus", System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic)!
        .Invoke(view, new object[] { Enum.ToObject(stateType, 2), 0u });
    var fcs = Activator.CreateInstance(fcsType, 1u, true, true, true, false, 133, 2, -.1f, 0f)!;
    viewType.GetMethod("Refresh")!.Invoke(view, new[] { target, fcs });
    if (!(bool)viewType.GetProperty("CanFire")!.GetValue(view)!)
        throw new Exception("Pinned GUI rejects ready asset state");
    Console.WriteLine("Actual pinned GUI AssignmentView: Assigned + operational asset enables the power button");
    return;
}
var json = new JsonSerializerOptions { IncludeFields = true };
if (Marshal.SizeOf<SharedFrame>() != 484 || Marshal.SizeOf<PipeCommand>() != 28 || Marshal.SizeOf<PipeResponse>() != 40)
    throw new Exception("GUI protocol layout mismatch");
using var reader = new SharedMemoryFrameReader();
using var client = new PipeCommandClient();
var gate = new object(); SharedFrame? latest = null;
reader.FrameReceived += f => { lock (gate) latest = f; };
reader.Start();
using var ready = new CancellationTokenSource(TimeSpan.FromSeconds(20));
while (true) {
    lock (gate) { if (latest is not null) break; }
    await Task.Delay(20, ready.Token);
}
Console.WriteLine("{\"ready\":true}");
while (Console.ReadLine() is string line && line != "quit") {
    if (line == "frame") {
        SharedFrame f; lock (gate) f = latest!.Value;
        Console.WriteLine(JsonSerializer.Serialize(f, json));
    } else {
        var q = JsonDocument.Parse(line).RootElement;
        using var timeout = new CancellationTokenSource(TimeSpan.FromSeconds(3));
        var result = await client.SendAsync((CommandType)q.GetProperty("command").GetUInt32(),
            q.GetProperty("target").GetUInt32(), cancellationToken: timeout.Token);
        Console.WriteLine(JsonSerializer.Serialize(result, json));
    }
}
