import { remultSveltekit } from "remult/remult-sveltekit";
import { Layout } from "../shared/Layout";
import { JsonDataProvider } from "remult";
import { JsonEntityFileStorage } from "remult/server";

export const api = remultSveltekit({
  entities: [Layout],
  admin: true, // enable admin UI
  // We always want to use the same json file so our config loads
  dataProvider: async () => new JsonDataProvider(new JsonEntityFileStorage(`${process.env.DEVENV_ROOT}/software/web_ui/db`))
});
