-- SPDX-License-Identifier: GPL-3.0-or-later
package body Simple_Loop is
   use AP_Module;
   use type Status;

   function Run (Ctx : Context) return Status is
      Result : Status;
   begin
      while not Stop_Requested (Ctx) loop
         Result := Send_Text (Ctx, Info, "Ada tick");
         if Result /= OK then
            return Result;
         end if;
         Result := Sleep_Ms (Ctx, 1000);
         if Result /= OK then
            return Result;
         end if;
      end loop;
      return Stopped;
   end Run;
end Simple_Loop;
