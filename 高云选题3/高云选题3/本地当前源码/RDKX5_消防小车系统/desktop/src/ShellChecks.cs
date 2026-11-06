using System;
using System.Reflection;
using System.Collections.Generic;

static class ShellChecks
{
    static int Main(string[] args)
    {
        var app = Assembly.LoadFrom(args[0]);
        var parser = app.GetType("RescueDesktop.PresentationForm").GetMethod("TryLiveAddress", BindingFlags.NonPublic | BindingFlags.Static);
        string[] accept = { "192.168.127.10:8090", "http://192.168.1.8:8090/", "10.137.137.171:8090", "172.16.0.1:8090", "172.31.255.1:8090", "127.0.0.1:8090", "169.254.1.2:8090" };
        string[] reject = { "https://192.168.1.8:8090", "192.168.127.10:8086", "8.8.8.8:8090", "localhost:8090", "127.1:8090", "2130706433:8090", "127.0.0.01:8090", "192.168.1.8:8090/path", "192.168.1.8:8090?x=1", "192.168.1.8:8090#x", "user@192.168.1.8:8090", "172.32.0.1:8090", "file:///C:/Windows", "javascript:alert(1)", "0.0.0.0:8090", "[::1]:8090", "192.168.1.8:8090\\evil" };
        int count = 0;
        foreach (var value in accept) { if (!(bool)parser.Invoke(null, new object[] { value, null })) throw new Exception("Expected accepted: " + value); count++; }
        foreach (var value in reject) { if ((bool)parser.Invoke(null, new object[] { value, null })) throw new Exception("Expected rejected: " + value); count++; }
        Console.WriteLine("PASS: " + count + " endpoint isolation cases");
        var liveCheck = app.GetType("RescueDesktop.PresentationForm").GetMethod("IsLiveState", BindingFlags.NonPublic | BindingFlags.Static);
        var real = new Dictionary<string, object> { { "mode", "live" }, { "synthetic", false } };
        if (!(bool)liveCheck.Invoke(null, new object[] { real })) throw new Exception("Live service rejected");
        var rejected = new Dictionary<string, object>[] {
            null,
            new Dictionary<string, object>(),
            new Dictionary<string, object> { { "mode", "demo" }, { "synthetic", true } },
            new Dictionary<string, object> { { "mode", "live" }, { "synthetic", true } },
            new Dictionary<string, object> { { "mode", "demo" }, { "synthetic", false } },
            new Dictionary<string, object> { { "mode", "live" } },
            new Dictionary<string, object> { { "synthetic", false } },
            new Dictionary<string, object> { { "mode", "live" }, { "synthetic", "false" } },
        };
        foreach (var state in rejected) if ((bool)liveCheck.Invoke(null, new object[] { state })) throw new Exception("Unverified or synthetic service accepted");
        Console.WriteLine("PASS: 9 real-service identity cases");
        return 0;
    }
}
